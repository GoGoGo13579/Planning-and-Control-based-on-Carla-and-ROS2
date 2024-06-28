#include "reference_line.h"

ReferenceLine::ReferenceLine()
{
    //初始化参考线
    _forward_num = 80;
    _backward_num = 15;
    _previous_match_point_index = 0;
    _is_first_run = true;

    //参考线平滑
    _smooth_solver = std::make_shared<OsqpEigen::Solver>();//创建求解器
    _smooth_solver->settings()->setWarmStart(true);//求解器热启动
    _cost_smooth = 5;//平滑代价
    _cost_geometry = 10;//几何相似代价
    _cost_compact = 50;//紧凑代价
    _max_x_offset = 0.5;//x方向最大偏移量
    _max_y_offset = 0.5;//y方向最大偏移量
}

bool ReferenceLine::run_step(std::shared_ptr<VehicleState> current_ego_state, const std::vector<PathPoint>& global_path,
                             std::vector<PathPoint>& reference_line)
{
    //1.搜索匹配点位置
    int match_point_index = 0;
    double min_distance = 1e10;
    int count_num = 0;
    if(_is_first_run)//首次运行
    {
        _is_first_run = false;
        for (int i = 0; i < (int)global_path.size(); i++)
        {
            double distance = std::hypot(current_ego_state->x - global_path.at(i).x, current_ego_state->y - global_path.at(i).y);
            if (distance < min_distance)
            {
                min_distance = distance;
                match_point_index = i;
                count_num = 0;
            }
            count_num++;
            if(count_num > 50)//向后50个都没有比它近的就先退出
            {
                break;
            }

        }
    }
    else//不是第一次运行，要判断检索方向
    {
        int start_index = _previous_match_point_index;
        PathPoint start_point;
        start_point = global_path.at(start_index);//搜索的起点
        Eigen::Vector2d tau(std::cos(start_point.heading), std::sin(start_point.heading));//起点的切向量
        Eigen::Vector2d start_point_to_ego(current_ego_state->x-start_point.x, current_ego_state->y-start_point.y);//起点到主车的向量
        double dot = (tau.transpose())*(start_point_to_ego.normalized());//点积判断方向

        if (dot > 1e-2)//向前搜索
        {
            for (int i = start_index; i < (int)global_path.size(); i++)
            {
                double distance = std::hypot(current_ego_state->x - global_path.at(i).x, current_ego_state->y - global_path.at(i).y);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    match_point_index = i;
                    count_num = 0;
                }
                count_num++;
                if(count_num > 30)
                {
                    break;
                }
            }
            
        }
        else if (dot < -1e-2)//向后搜索
        {
            for (int i = start_index; i >= 0; i--)
            {
                double distance = std::hypot(current_ego_state->x - global_path.at(i).x, current_ego_state->y - global_path.at(i).y);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    match_point_index = i;
                    count_num = 0;
                }
                count_num++;
                if(count_num > 30)
                {
                    break;
                }
            }
            
        }
        else//不用搜索
        {
            match_point_index = start_index;
        }
    }
    _previous_match_point_index = match_point_index;

    //2.获取初始参考线
    std::deque<PathPoint> init_reference_line;
    
    for (int i = 0; i <= _forward_num && i+match_point_index < (int)global_path.size(); i++)//匹配点在这个方向已经插入，不需要重入插入
    {
        init_reference_line.push_back(global_path.at(match_point_index + i));
    }

    for (int i = 1; i <= _backward_num && match_point_index-i >= 0; i++)
    {
        init_reference_line.push_front(global_path.at(match_point_index-i));
    }
    
    

    //3.平滑
    Eigen::SparseMatrix<double> H_sommth,H_geometry,H_compact,H;//H矩阵
    Eigen::VectorXd f;
    Eigen::SparseMatrix<double> A;
    Eigen::VectorXd lowbound, upbound;
    int var_num = init_reference_line.size();

    //3-1.H矩阵初始化
    H_sommth.resize(2*var_num - 4, 2*var_num);
    H_geometry.resize(2*var_num, 2*var_num);
    H_compact.resize(2*var_num - 2, 2*var_num);
    H.resize(2*var_num, 2*var_num);
    //H矩阵赋值
    for (int i = 0; i < var_num-2; i++)
    {
        H_sommth.insert(2*i, 2*i) = 1;
        H_sommth.insert(2*i, 2*i+2) = -2;
        H_sommth.insert(2*i, 2*i+4) = 1;
        H_sommth.insert(2*i+1, 2*i+1) = 1;
        H_sommth.insert(2*i+1, 2*i+3) = -2;
        H_sommth.insert(2*i+1, 2*i+5) = 1;
    }

    for (int i = 0; i < var_num-1; i++)
    {
        H_compact.insert(2*i, 2*i) = -1;
        H_compact.insert(2*i, 2*i+2) = 1;
        H_compact.insert(2*i+1, 2*i+1) = -1;
        H_compact.insert(2*i+1, 2*i+3) = 1;
    }

    for (int i = 0; i < 2* var_num; i++)
    {
        H_geometry.insert(i,i) = 1;
    }

    H = 2*(_cost_smooth*H_sommth.transpose()*H_sommth + _cost_compact*H_compact.transpose()*H_compact + _cost_geometry*H_geometry.transpose()*H_geometry);
    

    //3-2矩阵初始化
    f.resize(2*var_num);

    for (int i = 0; i < var_num; i++)
    {
        f[2*i] = init_reference_line.at(i).x;    
        f[2*i+1] = init_reference_line.at(i).y;    
    }

    f = -2*_cost_geometry*f;

    //3-3不等式矩阵初始化
    A.resize(2*var_num, 2*var_num);
    lowbound.resize(2*var_num);
    upbound.resize(2*var_num);

    for (int i = 0; i < var_num; i++)
    {
        A.insert(2*i,2*i) = 1;
        A.insert(2*i+1,2*i+1) = 1;
        lowbound(2*i) = init_reference_line.at(i).x - _max_x_offset;
        lowbound(2*i+1) = init_reference_line.at(i).y - _max_y_offset;
        upbound(2*i) = init_reference_line.at(i).x + _max_x_offset;
        upbound(2*i+1) = init_reference_line.at(i).y + _max_y_offset;
    }

    //3-4：设置求解器矩阵

    _smooth_solver->data()->setNumberOfVariables(2 * var_num);
    _smooth_solver->data()->setNumberOfConstraints(2 * var_num);
    if(!_smooth_solver->data()->setHessianMatrix(H)) {return false;}
    if(!_smooth_solver->data()->setGradient(f)) {return false;}
    if(!_smooth_solver->data()->setLinearConstraintsMatrix(A)) {return false;}
    if(!_smooth_solver->data()->setLowerBound(lowbound)) {return false;}
    if(!_smooth_solver->data()->setUpperBound(upbound)) {return false;}

    if(!_smooth_solver->initSolver()) {return false;}

    _smooth_solver->solveProblem();

    auto qp_solution = _smooth_solver->getSolution();


    //4.计算theta与kappa，输出
    reference_line.resize(var_num);
    for (int i = 0; i < var_num; i++)
    {
        reference_line.at(i).x = qp_solution(2*i);
        reference_line.at(i).y = qp_solution(2*i+1);
    }

    Calculate_heading_and_kappa(reference_line);

    _smooth_solver->data()->clearHessianMatrix();
    _smooth_solver->data()->clearLinearConstraintsMatrix();
    _smooth_solver->clearSolverVariables();
    _smooth_solver->clearSolver();
    return true;

 
}