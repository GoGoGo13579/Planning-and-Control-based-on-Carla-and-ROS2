#include "emplanner.h"


//这个采样太糙，不具有通用性，有改进的空间
void EMPlanner::get_path_dp_sample(std::vector<std::vector<FrenetPoint>>& path_dp_sample, const FrenetPoint& planning_start_sl, 
                    const double s_sample_distance, const size_t s_sample_number, 
                    const double l_sample_distance, const size_t l_sample_number)
{
    path_dp_sample.emplace_back();
    path_dp_sample.back().emplace_back(planning_start_sl);
    for (size_t level = 1; level <= s_sample_number; level++)
    {
        path_dp_sample.emplace_back();
        for (int i = 1; i <= (int)l_sample_number; i++)
        {
            FrenetPoint sl_point;
            sl_point.s = planning_start_sl.s + s_sample_distance * level;
            sl_point.l = l_sample_distance * (6-i);
            path_dp_sample.back().emplace_back(sl_point);
        }   
    }
}

double EMPlanner::calculate_dp_cost(const FrenetPoint& start_point, const FrenetPoint& end_point, const std::vector<FrenetPoint>& static_obstacles_sl_set,
                                    const WeightCoefficients& weight_coefficients)
{
    double cost = 0.0;
    int sample_num = 11;//采样点数目
    PolynomialCurve poly_curve;//获取曲线
    poly_curve.curve_fitting(start_point.s, start_point.l, start_point.l_prime, start_point.l_prime_prime, 
                             end_point.s, end_point.l, end_point.l_prime, end_point.l_prime_prime);
    //获取采样点函数值与各阶导数值，并储存
    auto linspace = Eigen::VectorXd::LinSpaced(sample_num,start_point.s,end_point.s);
    std::vector<double> s_set, l_set, dl_set, ddl_set, dddl_set;
    for (size_t i = 1; i < (size_t)linspace.size(); i++)//不包含起点
    {   
        s_set.emplace_back(linspace[i]);
        l_set.emplace_back(poly_curve.value_evaluation(linspace[i],0));
        dl_set.emplace_back(poly_curve.value_evaluation(linspace[i],1));
        ddl_set.emplace_back(poly_curve.value_evaluation(linspace[i],2));
        dddl_set.emplace_back(poly_curve.value_evaluation(linspace[i],3));
    }
    //参考线代价
    double cost_ref = 0.0;
    for (auto &&l : l_set)
    {
        cost_ref += weight_coefficients.path_dp_w_ref*l*l;
    }
    //平滑代价
    double cost_smooth = 0.0;
    for (size_t i = 0; i < dl_set.size(); i++)
    {
        cost_smooth += weight_coefficients.path_dp_w_dl*dl_set[i]*dl_set[i] + weight_coefficients.path_dp_w_ddl*ddl_set[i]*ddl_set[i] +
                       weight_coefficients.path_dp_w_dddl*dddl_set[i]*dddl_set[i];
    }
    //障碍物代价
    double cost_obs = 0.0;
    if (!static_obstacles_sl_set.empty())
    {
        for (size_t i = 0; i < static_obstacles_sl_set.size(); i++)
        {
            for (size_t j = 0; j < s_set.size(); j++)
            {
                double distance = std::hypot(s_set[j] - static_obstacles_sl_set[i].s, l_set[j] - static_obstacles_sl_set[i].l);
                if (distance >= 4)
                {
                    cost_obs += 0;
                }
                else if(distance <= 3)
                {
                    cost_obs += weight_coefficients.path_dp_w_obs;
                }
                else
                {
                    cost_obs += 1000.0/(distance*distance + 1e-6);
                }   
            }
        }
    }
    cost = cost_ref + cost_smooth + cost_obs;    
    return cost;
}

//加密动态规划的结果
void EMPlanner::increased_dp_path(const std::vector<FrenetPoint>& init_dp_path, const double increased_distance, std::vector<FrenetPoint>& final_dp_path)
{
    //对于dp产生的路径进行加密处理，即增加点的个数
    double sample_s = init_dp_path[1].s - init_dp_path[0].s;
    size_t num = (size_t)(sample_s / increased_distance);
    for (size_t i = 0; i < init_dp_path.size() - 1; i++)
    {
        PolynomialCurve sl_curve;
        sl_curve.curve_fitting(init_dp_path[i].s, init_dp_path[i].l, init_dp_path[i].l_prime, init_dp_path[i].l_prime_prime,
                               init_dp_path[i+1].s, init_dp_path[i+1].l, init_dp_path[i+1].l_prime, init_dp_path[i+1].l_prime_prime);
        for (size_t j = 0; j < num; j++)
        {
            double cur_s = init_dp_path[i].s + j*increased_distance;
            FrenetPoint cur_sl_point;
            cur_sl_point.s = cur_s;
            cur_sl_point.l = sl_curve.value_evaluation(cur_s,0);
            cur_sl_point.l_prime = sl_curve.value_evaluation(cur_s,1);
            cur_sl_point.l_prime_prime = sl_curve.value_evaluation(cur_s,2);
            final_dp_path.emplace_back(cur_sl_point);
        }
    }
    //因为前边是不包含每段最后一个点，随意整条路径的最后一个点单独加入
    final_dp_path.emplace_back(init_dp_path.back());

}

void EMPlanner::generate_convex_space(const std::vector<FrenetPoint>& final_dp_path, const double road_up_boundary, const double road_low_boundary,
                            const std::vector<FrenetPoint>& static_obs_sl_point_set, std::vector<double>& path_l_max, std::vector<double>& path_l_min)
{
    double obs_length = 5.0;//障碍物长度
    double obs_width = 3.0;//障碍物宽度
    for (size_t i = 0; i < final_dp_path.size(); i++)
    {
        path_l_max.emplace_back(road_up_boundary);
        path_l_min.emplace_back(road_low_boundary);
    }

    if (!static_obs_sl_point_set.empty())
    {
        for (size_t i = 0; i < static_obs_sl_point_set.size(); i++)
        {
            //获取离障碍物最近(指的是横向距离最小)的路径点索引
            int center_index = find_index_for_obs_on_path(final_dp_path, static_obs_sl_point_set[i].s);
            if (center_index == -1)
            {
                continue;
            }
            if (final_dp_path[center_index].l > static_obs_sl_point_set[i].l)//左侧绕行
            {
                int start_index = find_index_for_obs_on_path(final_dp_path, static_obs_sl_point_set[i].s - obs_length/2.0);
                int end_index = find_index_for_obs_on_path(final_dp_path, static_obs_sl_point_set[i].s + obs_length/2.0);
                if (start_index == -1 || end_index == -1)//索引值等于-1表示在界外
                {
                    continue;
                }
                for (int j = start_index; j <= end_index; j++)
                {
                    path_l_min[j] = std::max(path_l_min[j], static_obs_sl_point_set[i].l + obs_width/2.0);
                }
                
            }
            else//右侧绕行
            {
                int start_index = find_index_for_obs_on_path(final_dp_path, static_obs_sl_point_set[i].s - obs_length/2.0);
                int end_index = find_index_for_obs_on_path(final_dp_path, static_obs_sl_point_set[i].s + obs_length/2.0);
                if (start_index == -1 || end_index == -1)//索引值等于-1表示在界外
                {
                    continue;
                }
                for (int j = start_index; j <= end_index; j++)
                {
                    path_l_max[j] = std::min(path_l_max[j], static_obs_sl_point_set[i].l - obs_width/2.0);
                }
            } 
        }
    }
}


int EMPlanner::find_index_for_obs_on_path(const std::vector<FrenetPoint>& final_dp_path,  const double static_obs_s)
{
    int index = -1;
    if (static_obs_s < final_dp_path.front().s)
    {
        index = 0;
    }
    else if(static_obs_s >= final_dp_path.back().s)
    {
        index = (int)final_dp_path.size()-1;
    }
    else
    {
        for (size_t i = 0; i < final_dp_path.size() - 1; i++)
        {
            if (static_obs_s >= final_dp_path[i].s && static_obs_s < final_dp_path[i+1].s)
            {
                if (std::abs(static_obs_s - final_dp_path[i].s) >= std::abs(final_dp_path[i+1].s - static_obs_s))
                {
                    index = i+1;
                    break;
                }
                else
                {
                    index = i;
                    break;
                }
            }
        }
    }
    return index;    
}

bool EMPlanner::path_QP_planning(const std::vector<FrenetPoint>& final_dp_path, const std::vector<double>& final_dp_path_lmin, const std::vector<double>& final_dp_path_lmax, 
                      const double l_desire, const double dl_desire, const double ddl_desire,
                      const WeightCoefficients& weight_coeficients, std::vector<FrenetPoint>& init_qp_path )
{
    size_t point_num = final_dp_path.size();

    //------------------建立目标函数------------------
    //1.1H矩阵
    Eigen::SparseMatrix<double> H_ref, H_dl, H_ddl, H_dddl, H_mid, H_l_end, H_dl_end, H_ddl_end, H;

    H_ref.resize(point_num, 3*point_num);
    H_dl.resize(point_num, 3*point_num);
    H_ddl.resize(point_num, 3*point_num);
    H_mid.resize(point_num, 3*point_num);
    for (size_t i = 0; i < point_num; i++)
    {
        H_ref.insert(i, 3*i) = 1;
        H_mid.insert(i, 3*i) = 1;
        H_dl.insert(i, 3*i+1) = 1;
        H_ddl.insert(i, 3*i+2) = 1;
    }

    H_dddl.resize(point_num-1, 3*point_num);
    for (size_t i = 0; i < point_num-1; i++)
    {
        H_dddl.insert(i, 3*i+2) = -1;
        H_dddl.insert(i, 3*(i+1) + 2) = 1;
    }

    H_l_end.resize(1, 3*point_num);
    H_dl_end.resize(1, 3*point_num);
    H_ddl_end.resize(1, 3*point_num);
    H_l_end.insert(0, 3*(point_num-1)) = 1;
    H_dl_end.insert(0, 3*(point_num-1)+1) = 1;
    H_ddl_end.insert(0, 3*(point_num-1)+2) = 1;

    H = 2*(weight_coeficients.path_qp_w_ref     *  H_ref.transpose()     *  H_ref     + 
           weight_coeficients.path_qp_w_dl      *  H_dl.transpose()      *  H_dl      + 
           weight_coeficients.path_qp_w_ddl     *  H_ddl.transpose()     *  H_ddl     +
           weight_coeficients.path_qp_w_dddl    *  H_dddl.transpose()    *  H_dddl    +
           weight_coeficients.path_qp_w_mid     *  H_mid.transpose()     *  H_mid     +
           weight_coeficients.path_qp_w_l_end   *  H_l_end.transpose()   *  H_l_end   +
           weight_coeficients.path_qp_w_dl_end  *  H_dl_end.transpose()  *  H_dl_end  + 
           weight_coeficients.path_qp_w_ddl_end *  H_ddl_end.transpose() *  H_ddl_end   );

    //1.2f矩阵
    Eigen::VectorXd f_mid, f_l_end, f_dl_end, f_ddl_end, f;
    f_mid.setZero(3*point_num);
    f_l_end.setZero(3*point_num);
    f_dl_end.setZero(3*point_num);
    f_ddl_end.setZero(3*point_num);
    for (size_t i = 0; i < point_num; i++)
    {
        //版本1
        f_mid[3*i] = -2.0 * (final_dp_path_lmax[i] + final_dp_path_lmin[i])/2.0;
        //版本2 
        //f_mid[3*i] = -2.0 * final_dp_path[i].l;
    }
    f_l_end[3*(point_num-1)] = -2.0*l_desire;
    f_dl_end[3*(point_num-1) + 1] = -2.0*dl_desire;
    f_ddl_end[3*(point_num-1) + 2] = -2.0*ddl_desire;

    f.resize(3*point_num);
    f = weight_coeficients.path_qp_w_mid * f_mid + 
        weight_coeficients.path_qp_w_l_end * f_l_end +
        weight_coeficients.path_qp_w_dl_end * f_dl_end + 
        weight_coeficients.path_qp_w_ddl_end * f_ddl_end;
    

    //------------------建立约束------------------
    //2.1等式约束(连续性约束),由于osqp-eigen的约束形式是统一的，l <= Ax <= u,所以等式约束要做出相应改变
    //由原来的Ax = b编程 b <= Ax <= b 
    Eigen::SparseMatrix<double> A_continuity;
    A_continuity.resize(2*(point_num-1), 3*point_num);
    double delta_s = final_dp_path[1].s - final_dp_path[0].s;
    for (size_t i = 0; i < point_num-1; i++)
    {
        A_continuity.insert(2*i, 3*i + 0) = 1.0;
        A_continuity.insert(2*i, 3*i + 1) = delta_s;
        A_continuity.insert(2*i, 3*i + 2) = delta_s*delta_s/3.0;
        A_continuity.insert(2*i, 3*i + 3) = -1.0;
        A_continuity.insert(2*i, 3*i + 4) = 0.0;
        A_continuity.insert(2*i, 3*i + 5) = delta_s*delta_s/6.0;
        A_continuity.insert(2*i + 1, 3*i + 0) = 0.0;
        A_continuity.insert(2*i + 1, 3*i + 1) = 1.0;
        A_continuity.insert(2*i + 1, 3*i + 2) = delta_s/2.0;
        A_continuity.insert(2*i + 1, 3*i + 3) = 0.0;
        A_continuity.insert(2*i + 1, 3*i + 4) = -1.0;
        A_continuity.insert(2*i + 1, 3*i + 5) = delta_s/2.0;
    }

    Eigen::VectorXd low_boundary_continuity, up_boundary_continuity;
    low_boundary_continuity.setZero(2*point_num -2);
    up_boundary_continuity.setZero(2*point_num -2);


    //2.2不等式约束(碰撞约束)
    Eigen::SparseMatrix<double> A_collision_avoidance;
    A_collision_avoidance.resize(4*point_num, 3*point_num);
    double d1 = 3;
    double d2 = 2;
    double car_width = 3;
    for (size_t i = 0; i < point_num; i++)
    {
        A_collision_avoidance.insert(4*i + 0, 3*i + 0) = 1;
        A_collision_avoidance.insert(4*i + 0, 3*i + 1) = d1;
        A_collision_avoidance.insert(4*i + 0, 3*i + 2) = 0;
        A_collision_avoidance.insert(4*i + 1, 3*i + 0) = 1;
        A_collision_avoidance.insert(4*i + 1, 3*i + 1) = d1;
        A_collision_avoidance.insert(4*i + 1, 3*i + 2) = 0;
        A_collision_avoidance.insert(4*i + 2, 3*i + 0) = 1;
        A_collision_avoidance.insert(4*i + 2, 3*i + 1) = -d2;
        A_collision_avoidance.insert(4*i + 2, 3*i + 2) = 0;
        A_collision_avoidance.insert(4*i + 3, 3*i + 0) = 1;
        A_collision_avoidance.insert(4*i + 3, 3*i + 1) = -d2;
        A_collision_avoidance.insert(4*i + 3, 3*i + 2) = 0;
    }
    
    Eigen::VectorXd low_boundary_collision_avoidance, up_boundary_collision_avoidance;
    low_boundary_collision_avoidance.setZero(4*point_num);
    up_boundary_collision_avoidance.setZero(4*point_num);
    for (size_t i = 0; i < point_num; i++)
    {
        int start_index = (int)std::max(floor(i - d2), 0.0);
        int end_index = std::min((int)ceil(i + d1), (int)point_num-1);
        double ubi = std::numeric_limits<double>::max();//在start_index到end_index中lmax里最小的
        double lbi = std::numeric_limits<double>::lowest();//在start_index到end_index中lmin里最大的

        for(int j = start_index ; j <= end_index ; j++ )
        {
            if (final_dp_path_lmax[j] < ubi)
            {
                ubi = final_dp_path_lmax[j];
            }
            if (final_dp_path_lmin[j] > lbi)
            {
                lbi = final_dp_path_lmin[j];
               // std::cout << lbi << " " << final_dp_path_lmin[j] << std::endl;
            }
        }
    
        low_boundary_collision_avoidance[4*i + 0] = lbi - car_width/2.0;
        low_boundary_collision_avoidance[4*i + 1] = lbi + car_width/2.0;
        low_boundary_collision_avoidance[4*i + 2] = lbi - car_width/2.0;
        low_boundary_collision_avoidance[4*i + 3] = lbi + car_width/2.0;
        up_boundary_collision_avoidance[4*i + 0] = ubi - car_width/2.0;
        up_boundary_collision_avoidance[4*i + 1] = ubi + car_width/2.0;
        up_boundary_collision_avoidance[4*i + 2] = ubi - car_width/2.0;
        up_boundary_collision_avoidance[4*i + 3] = ubi + car_width/2.0;
    }
    low_boundary_collision_avoidance[0] = std::numeric_limits<double>::lowest();
    low_boundary_collision_avoidance[1] = std::numeric_limits<double>::lowest();
    low_boundary_collision_avoidance[2] = std::numeric_limits<double>::lowest();
    low_boundary_collision_avoidance[3] = std::numeric_limits<double>::lowest();
    up_boundary_collision_avoidance[0] = std::numeric_limits<double>::max();
    up_boundary_collision_avoidance[1] = std::numeric_limits<double>::max();
    up_boundary_collision_avoidance[2] = std::numeric_limits<double>::max();
    up_boundary_collision_avoidance[3] = std::numeric_limits<double>::max();

    //2.3规划起点约束
    Eigen::SparseMatrix<double> A_start_point;
    Eigen::Vector3d up_boundary_start_point, low_boundary_start_point;
    A_start_point.resize(3, 3*point_num);
    A_start_point.insert(0,0) = 1.0;
    A_start_point.insert(1,1) = 1.0;
    A_start_point.insert(2,2) = 1.0;
    up_boundary_start_point[0] = final_dp_path.front().l;
    up_boundary_start_point[1] = final_dp_path.front().l_prime;
    up_boundary_start_point[2] = final_dp_path.front().l_prime_prime;
    low_boundary_start_point = up_boundary_start_point;

    //2.4把所有约束拼起来
    Eigen::SparseMatrix<double> A_total, A_total_transpose;
    Eigen::VectorXd up_boundary_total, low_boundary_total;
    A_total.resize(A_continuity.rows() + A_collision_avoidance.rows() + A_start_point.rows(), 3*point_num);
    A_total_transpose.resize(3*point_num, A_continuity.rows() + A_collision_avoidance.rows() + A_start_point.rows());
    //对于主列（默认）的稀疏矩阵，是不允许用middlerows来赋值的，所以咱们只能先转置，在赋值，在转置回来，曲线救国
    A_total_transpose.middleCols(0, A_continuity.rows()) = A_continuity.transpose();
    //std::cout << A_total_transpose << std::endl;
    A_total_transpose.middleCols(A_continuity.rows(), A_collision_avoidance.rows()) = A_collision_avoidance.transpose();
    //std::cout << A_total_transpose << std::endl;
    A_total_transpose.middleCols(A_continuity.rows() + A_collision_avoidance.rows(), A_start_point.rows()) = A_start_point.transpose();
    //std::cout << A_total_transpose << std::endl;
    A_total = A_total_transpose.transpose();

    up_boundary_total.resize(up_boundary_continuity.rows() + up_boundary_collision_avoidance.rows() + up_boundary_start_point.rows());
    low_boundary_total.resize(low_boundary_continuity.rows() + low_boundary_collision_avoidance.rows() + low_boundary_start_point.rows());
    up_boundary_total << up_boundary_continuity, up_boundary_collision_avoidance, up_boundary_start_point;
    low_boundary_total << low_boundary_continuity, low_boundary_collision_avoidance, low_boundary_start_point;
    
    //-----------------求解-------------------------------
    //3.1初始化


    _path_qp_solver.data()->setNumberOfVariables(3*point_num);
    _path_qp_solver.data()->setNumberOfConstraints(6*point_num + 1);//这里可以用A_total的行数，但是可以用这个数做个验证，如果对不上就是哪里写错了
    if(!_path_qp_solver.data()->setHessianMatrix(H)) {return false;}
    if(!_path_qp_solver.data()->setLinearConstraintsMatrix(A_total)) {return false;}
    if(!_path_qp_solver.data()->setGradient(f)) {return false;}
    if(!_path_qp_solver.data()->setBounds(low_boundary_total, up_boundary_total)) {return false;}
    if(!_path_qp_solver.initSolver()){return false;} 
    //3.2求解并赋初值
    if(!(_path_qp_solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError)){return false;}
    auto solution = _path_qp_solver.getSolution();
    for (size_t i = 0; i < point_num; i++)
    {
        FrenetPoint qp_path_point;
        qp_path_point.s = final_dp_path[i].s;
        qp_path_point.l = solution[3*i + 0];
        qp_path_point.l_prime = solution[3*i + 1];
        qp_path_point.l_prime_prime = solution[3*i + 2];
       // std::cout << qp_path_point.s << " " << qp_path_point.l << " " << std::endl;
        init_qp_path.emplace_back(qp_path_point);
    }

    _path_qp_solver.data()->clearHessianMatrix();
    _path_qp_solver.data()->clearLinearConstraintsMatrix();
    _path_qp_solver.clearSolverVariables();
    _path_qp_solver.clearSolver();

    // std::cout << "----------------目标函数矩阵-----------------"<< std::endl;
    // std::cout << "****************H_ref************************"<< std::endl;
    // std::cout << H_ref << std::endl;
    // std::cout << "****************H_dl************************"<< std::endl;
    // std::cout << H_dl << std::endl;
    // std::cout << "****************H_ddl************************"<< std::endl;
    // std::cout << H_ddl << std::endl;
    // std::cout << "****************H_dddl************************"<< std::endl;
    // std::cout << H_dddl << std::endl;
    // std::cout << "****************H_mid************************"<< std::endl;
    // std::cout << H_mid << std::endl;
    // std::cout << "****************H_l_end************************"<< std::endl;
    // std::cout << H_l_end << std::endl;
    // std::cout << "****************H_dl_end************************"<< std::endl;
    // std::cout << H_dl_end << std::endl;
    // std::cout << "****************H_ddl_end************************"<< std::endl;
    // std::cout << H_ddl_end << std::endl;
    // std::cout << "****************H************************"<< std::endl;
    // std::cout << H << std::endl;

    // std::cout << "------------------------f_mid----------------------" << std::endl;
    // std::cout << f_mid << std::endl;
    // std::cout << "------------------------f_l_end----------------------" << std::endl;
    // std::cout << f_l_end << std::endl;
    // std::cout << "------------------------f_dl_end----------------------" << std::endl;
    // std::cout << f_dl_end << std::endl;
    // std::cout << "------------------------f_ddl_end----------------------" << std::endl;
    // std::cout << f_ddl_end << std::endl;

    // std::cout << "----------------连续性矩阵-----------------"<< std::endl;
    // std::cout << A_continuity << std::endl;
    // std::cout << "----------------连续性上界-----------------"<< std::endl;
    // std::cout << up_boundary_continuity << std::endl;
    // std::cout << "----------------连续性下界-----------------"<< std::endl;
    // std::cout << low_boundary_continuity << std::endl;

    // std::cout << "----------------碰撞矩阵-----------------"<< std::endl;
    // std::cout << A_collision_avoidance << std::endl;
    // std::cout << "----------------碰撞上界-----------------"<< std::endl;
    // std::cout << up_boundary_collision_avoidance << std::endl;
    // std::cout << "----------------凸空间上界-----------------"<< std::endl;
    // for (auto &&i : final_dp_path_lmax)
    // {
    //     std::cout << i << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "----------------碰撞下界-----------------"<< std::endl;
    // std::cout << low_boundary_collision_avoidance << std::endl;
    // std::cout << "----------------凸空间下界-----------------"<< std::endl;
    // for (auto &&i : final_dp_path_lmin)
    // {
    //     std::cout << i << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "----------------起点矩阵-----------------"<< std::endl;
    // std::cout << A_start_point << std::endl;
    // std::cout << "----------------起点上界-----------------"<< std::endl;
    // std::cout << up_boundary_start_point << std::endl;
    // std::cout << "----------------起点下界-----------------"<< std::endl;
    // std::cout << low_boundary_start_point << std::endl;
    
    // std::cout <<"----------------------最优解------------------" <<std::endl;
    // for (auto &&i : init_qp_path)
    // {
    //     std::cout << "("<< i.s << ", " << i.l << ", "<< i.l_prime << ", " << i.l_prime_prime << ")" << " ";
    // }
    // std::cout << std::endl;

    std::cout << "参考线代价:" << weight_coeficients.path_qp_w_ref * 2* solution.transpose() * H_ref.transpose() * H_ref * solution << std::endl;
    std::cout << "dl代价:" <<  weight_coeficients.path_qp_w_dl * 2 * solution.transpose() * H_dl.transpose() * H_dl * solution << std::endl;
    std::cout << "ddl代价:" << weight_coeficients.path_qp_w_ddl * 2 * solution.transpose() * H_ddl.transpose() * H_ddl * solution << std::endl;   
    std::cout << "dddl代价:" <<  weight_coeficients.path_qp_w_dddl * 2* solution.transpose() * H_dddl.transpose() * H_dddl * solution << std::endl;
    std::cout << "mid代价:" <<  weight_coeficients.path_qp_w_mid * (2* solution.transpose() * H_mid.transpose() * H_mid * solution  + f_mid.transpose() * solution)<< std::endl;
 
    // std::cout <<"---------------------H矩阵-------------------------" << std::endl;
    // std::cout << H << std::endl;
    // std::cout <<"---------------------f矩阵-------------------------" << std::endl;
    // std::cout << f << std::endl;
    // std::cout <<"---------------------A矩阵-------------------------" << std::endl;
    // std::cout << A_total << std::endl;
    // std::cout <<"---------------------约束上界矩阵-------------------------" << std::endl;
    // std::cout << up_boundary_total << std::endl;
    // std::cout <<"---------------------约束下界矩阵-------------------------" << std::endl;
    // std::cout << low_boundary_total << std::endl;
    
    return true;
}






