#include "emplanner.h"

 void EMPlanner::generate_st_graph(const std::vector<FrenetPoint>& dynamic_obs_sl_set, const double delat_l, std::vector<std::unordered_map<std::string, double>>& st_graph)
 {
    if (dynamic_obs_sl_set.empty())
    {
        return;
    }

    for (auto &&obs : dynamic_obs_sl_set)
    {
        double t_in, t_out;
        if (std::abs(obs.l_dot) <= 0.2)//对于速度过慢的障碍物，且在上下边界内，应该考虑作为虚拟障碍物
        {
            continue;
        }
        else
        {
            if (std::abs(obs.l) > delat_l) //动态障碍物初始位于界外
            {
                if (obs.l * obs.l_dot > 0) //界外，且在上侧时向上开，在下侧时向下开，此时没有危险
                {
                    continue;
                }
                else//界外，且在上侧时向下开，在下侧时向上开，此时有危险
                {
                    t_in = std::abs(obs.l/obs.l_dot) - std::abs(delat_l/obs.l_dot);
                    t_out = std::abs(obs.l/obs.l_dot) + std::abs(delat_l/obs.l_dot);
                }
            }
            else //动态障碍物初始位于界内
            {
                t_in = 0.0;
                if (obs.l_dot > 0) //界内且向上开
                {
                    t_out = (delat_l - obs.l)/obs.l_dot;
                }
                else//界内且向下开
                {
                    t_out = (-delat_l - obs.l)/obs.l_dot;
                }
            }
        }

        if (t_out >= 8 || t_out <= 1)//碰瓷或者太远，不处理
        {
            continue;
        }
        else
        {
            st_graph.emplace_back();
            st_graph.back()["t_in"] = t_in;
            st_graph.back()["t_out"] = t_out;
            st_graph.back()["s_in"] = obs.s + obs.s_dot * t_in;
            st_graph.back()["s_out"] = obs.s + obs.s_dot * t_out;
        }
    }
 }

STPoint EMPlanner::calculate_speed_start_point(const TrajectoryPoint& planning_start_point)
{
    //不能用直接规划起点的sl坐标，因为那是以参考线为s轴的，现在换轴了。
    STPoint planning_start_point_st;
    planning_start_point_st.t = 0.0;
    planning_start_point_st.s = 0.0;
    Eigen::Vector2d a(planning_start_point.ax, planning_start_point.ay);
    Eigen::Vector2d tau(std::cos(planning_start_point.heading), std::sin(planning_start_point.heading));
    planning_start_point_st.s_dot = planning_start_point.v;
    planning_start_point_st.s_dot_dot = a.dot(tau);

    return planning_start_point_st;
}


void EMPlanner::get_speed_dp_sample(const STPoint& planning_start_point_st_coordinate, const double t_sample_distance, const double t_end, const std::vector<double>& index2s,std::vector<std::vector<STPoint>>& speed_dp_sample)
{
    int t_sample_number = ceil(t_end/t_sample_distance);
    double s_end = index2s.back();//s的最大值
    std::deque<double> s_sample;//s的采样表
    s_sample.emplace_front(0.0);
    //s的采样最开始0.5m采样一个，每隔10个采样点增长一倍
    int count = 1;
    double delta_s = 0.5;
    while (true)
    {
        double next_s = s_sample.front() + delta_s;
        if (next_s > s_end)
        {
            break;
        }
        s_sample.emplace_front(next_s);//向前插入是想保证越靠前的点s值越大
        count++;
        if (count % 10 == 1)
        {
            delta_s *= 2;
        }
    }

    speed_dp_sample.emplace_back();
    speed_dp_sample.back().emplace_back(planning_start_point_st_coordinate);
    for (int level = 1; level <= t_sample_number; level++)
    {
        speed_dp_sample.emplace_back();
        for (size_t i = 0; i < s_sample.size(); i++)
        {
            STPoint sample_point;
            sample_point.s = s_sample[i];
            sample_point.t = level * t_sample_distance;
            speed_dp_sample.back().emplace_back(sample_point);   
        }
    }
}

double EMPlanner::calculate_speed_dp_cost(const STPoint& start_point, const STPoint& end_point, const double reference_speed,
                               const std::vector<std::unordered_map<std::string, double>>& dynamic_obs_st_graph, const WeightCoefficients& weight_coefficients)
{
    //1.参考速度代价
    double delta_t = end_point.t - start_point.t;
    double s_dot = (end_point.s - start_point.s) / delta_t;
    double cost_ref_speed = weight_coefficients.speed_dp_w_ref_speed*(s_dot - reference_speed)*(s_dot - reference_speed);

    //2.加速度代价
    double s_dot_dot = (s_dot - start_point.s_dot) / delta_t;
    double cost_a = 0.0;
    if (s_dot_dot > 4 || s_dot_dot < -6)
    {
        cost_a += 1e6;
    }
    else
    {
        cost_a = weight_coefficients.speed_dp_w_a * s_dot_dot *s_dot_dot;
    }
    

    //3.jerk代价
    double s_dot_dot_dot = (s_dot_dot - start_point.s_dot_dot) / delta_t;
    double cost_jerk = weight_coefficients.speed_dp_w_jerk * s_dot_dot_dot * s_dot_dot_dot;

    //4.障碍物代价
    double cost_obs = 0.0;

    if (!dynamic_obs_st_graph.empty())
    {
        //4.1获取采样点
        int n = 6;
        auto linspace = Eigen::VectorXd::LinSpaced(n, start_point.t, end_point.t);
        std::vector<double> t_set, s_set;
        for (int i = 1; i < linspace.size(); i++)//获取采样点
        {
            t_set.emplace_back(linspace[i]);
            s_set.emplace_back(start_point.s + s_dot*(t_set.back() - start_point.t));
        }
        //逐个障碍物计算代价
        for (auto && obs : dynamic_obs_st_graph)
        {
            for (size_t i = 0; i < t_set.size(); i++)
            {
                double min_distance = 0.0;
                Eigen::Vector2d in_to_out(obs.at("t_out") - obs.at("t_in"), obs.at("s_out")-obs.at("s_in"));
                Eigen::Vector2d sample_point_to_in(obs.at("t_in") - t_set[i], obs.at("s_in") - s_set[i]);
                Eigen::Vector2d sample_point_to_out(obs.at("t_out") - t_set[i], obs.at("s_out") - s_set[i]);
                if ((sample_point_to_in.dot(in_to_out))*(sample_point_to_out.dot(in_to_out)) >= 0)
                {
                    min_distance = std::min(sample_point_to_in.norm(), sample_point_to_out.norm());
                }
                else
                {
                    min_distance = std::abs(sample_point_to_in[0]*sample_point_to_out[1] - sample_point_to_in[1]*sample_point_to_out[0])/in_to_out.norm();
                }
                //由距离计算代价
                if (min_distance > 3)
                {
                    cost_obs += 0;
                }
                else if (min_distance < 2)
                {
                    cost_obs += weight_coefficients.speed_dp_w_obs;
                }
                else
                {
                    cost_obs += 1000/(min_distance*min_distance);
                }
            }
        }
    }
    double cost_total = cost_ref_speed + cost_a + cost_jerk +cost_obs;
    return cost_total; 
}

void EMPlanner::generate_convex_space(
                const std::vector<TrajectoryPoint>& init_trajectory, 
                const std::vector<double> path_index2s,
                const std::deque<STPoint>& speed_profile, 
                const std::vector<std::unordered_map<std::string, double>>& dynamic_obs_st_graph, 
                const double& ay_max, 
                std::vector<double>& s_lb, std::vector<double>& s_ub, 
                std::vector<double>& s_dot_lb, std::vector<double>& s_dot_ub) 
{
    s_lb.emplace_back(speed_profile.front().s);
    s_ub.emplace_back(speed_profile.front().s);
    s_dot_lb.emplace_back(speed_profile.front().s_dot);
    s_dot_ub.emplace_back(speed_profile.front().s_dot);
    //1确定s_dot的上下界
    for (size_t i = 1; i < speed_profile.size(); i++)
    {
        //1.1计算当前点的曲率
        int obs_s_index = -1;
        double cur_s = speed_profile[i].s;
        double cur_kappa;
        if (cur_s == path_index2s.back())
        {
            obs_s_index = (int)init_trajectory.size()-1;
            cur_kappa = init_trajectory.back().kappa;
        }
        else
        {   //因为speed_profile的点比较密，所以线性插值计算
            for (int j = 0; j < (int)path_index2s.size()-1; j++)
            {
                if (cur_s >= path_index2s[j] && cur_s < path_index2s[j+1])
                {
                    obs_s_index = j;
                    break;
                }
            }
            double k =  (init_trajectory[obs_s_index+1].kappa - init_trajectory[obs_s_index].kappa)/(path_index2s[obs_s_index+1] - path_index2s[obs_s_index]);
            cur_kappa = init_trajectory[obs_s_index].kappa + k*(cur_s - path_index2s[obs_s_index]);
        }
        
        //1.2由曲率计算速度上下界
        s_dot_ub.emplace_back(std::sqrt(std::abs(ay_max/cur_kappa)));
        s_dot_lb.emplace_back(0.0);
    }

    //2确定s的上下界
    for (size_t i = 1; i < speed_profile.size(); i++)
    {
        s_ub.emplace_back(path_index2s.back());
        s_lb.emplace_back(0.0);
    }
    
    for (auto && cur_obs : dynamic_obs_st_graph)
    {
        if ( cur_obs.at("t_in") <= speed_profile[1].t )//碰瓷行为，处理不了。拿咱们这个来说，就是0.5s之内突然出现一个障碍物，那就直接撞上去了。
        {
            continue;
        }
        //2.1判断超车还是减速
        //直接使用st图的中点判断
        double obs_t_center = (cur_obs.at("t_in") + cur_obs.at("t_out"))/2.0;
        double obs_s_center = (cur_obs.at("s_in") + cur_obs.at("s_out"))/2.0;
        int obs_t_center_index = find_t_index(speed_profile, obs_t_center);
        if (obs_t_center_index == -1)
        {//超界情况，不做处理
            continue;
        }
        //2.2判断并修改相应的上下界
        int start_index = find_t_index(speed_profile, cur_obs.at("t_in"));
        int end_index = find_t_index(speed_profile, cur_obs.at("t_out"));
        //对起始点和截至点做偏向于安全的放缩
        start_index = std::max(start_index-2, 1);//第一个不处理，因为是规划起点，是等式约束
        end_index = std::min(end_index + 2, (int)speed_profile.size()-1);
        double k = (cur_obs.at("s_out")-cur_obs.at("s_in"))/(cur_obs.at("t_out")-cur_obs.at("t_in"));
        if (speed_profile[obs_t_center_index].s >= obs_s_center)//加速超车
        {
            for (int i = start_index; i <= end_index; i++)
            {
                s_lb[i] = std::max(s_lb[i],obs_s_center + k*(speed_profile[i].t)-obs_t_center);
            }
            
        }
        else//减速让行
        {
            for (int i = start_index; i <= end_index; i++)
            {
                s_ub[i] = std::min(s_ub[i],obs_s_center + k*(speed_profile[i].t)-obs_t_center);
            }
        }
    }
}

int EMPlanner::find_t_index(const std::deque<STPoint>& speed_profile, const double t)
{
    int index = -1;
    if (t >= speed_profile.back().t || t < speed_profile.front().t)//超界情况
    {
        index = -1;
    }
    else//在界内,左边界在进入这个函数之前判断是否碰瓷里已经淘汰掉了
    {
        for (int j = 0; j < (int)speed_profile.size()-1; j++)
        {      
            if (t >= speed_profile[j].t && t < speed_profile[j+1].t)
            {
               if (std::abs(t-speed_profile[j].t) <= std::abs(t-speed_profile[j+1].t))
               {
                    index = j;
                    break;
               }
               else
               {
                    index = j+1;
                    break;
               }
            }
        }
    }
    return index;
}

bool EMPlanner::speed_QP_planning(const std::deque<STPoint>& dp_speed_profile, const std::vector<double>& s_lb, const std::vector<double>& s_ub, const double reference_speed, 
                        const std::vector<double>& s_dot_lb, const std::vector<double>& s_dot_ub, const WeightCoefficients& weight_coeficients, std::vector<STPoint>& qp_speed_profile)
{
    size_t point_num = dp_speed_profile.size();

    //----------------建立目标函数-------------------
    //1.1H矩阵
    Eigen::SparseMatrix<double> H_v, H_a, H_jerk, H_total;
    H_v.resize(point_num, 3*point_num);
    H_a.resize(point_num, 3*point_num);
    H_jerk.resize(point_num-1, 3*point_num);
    for (size_t i = 0; i < point_num; i++)
    {
        H_v.insert(i, 3*i + 1) = 1;
        H_a.insert(i, 3*i + 2) = 1;
    }
    for (size_t i = 0; i < point_num -1; i++)
    {
        H_jerk.insert(i, 3*i + 2) = -1;
        H_jerk.insert(i, 3*(i+1) + 2) = 1;
    }

    H_total.resize(point_num, point_num);

    H_total = 2*(weight_coeficients.speed_qp_w_ref_speed  *  H_v.transpose()    *    H_v + 
                 weight_coeficients.speed_qp_w_a          *  H_a.transpose()    *    H_a +
                 weight_coeficients.speed_qp_w_jerk       *  H_jerk.transpose() *    H_jerk);
    
    //1.2f矩阵
    Eigen::VectorXd f;
    f.setZero(3*point_num);
    for (size_t i = 0; i < point_num; i++)
    {
        f[3*i + 1] = -2 * weight_coeficients.speed_qp_w_ref_speed * reference_speed;
    }

    //---------------------建立约束------------------------
    //2.1连续性约束
    Eigen::SparseMatrix<double> A_continuity;
    Eigen::VectorXd low_boundary_continuity, up_boundary_continuity;
    double delta_t = dp_speed_profile[1].t - dp_speed_profile[0].t;

    A_continuity.resize(2*point_num - 2, 3*point_num);
    for (size_t i = 0; i < point_num - 1; i++)
    {
        A_continuity.insert(2*i + 0, 3*i + 0) = 1.0;
        A_continuity.insert(2*i + 0, 3*i + 1) = delta_t;
        A_continuity.insert(2*i + 0, 3*i + 2) = delta_t*delta_t/3.0;
        A_continuity.insert(2*i + 0, 3*i + 3) = -1.0;
        A_continuity.insert(2*i + 0, 3*i + 4) = 0.0;
        A_continuity.insert(2*i + 0, 3*i + 5) = delta_t*delta_t/6.0;
        A_continuity.insert(2*i + 1, 3*i + 0) = 0.0;
        A_continuity.insert(2*i + 1, 3*i + 1) = 1.0;
        A_continuity.insert(2*i + 1, 3*i + 2) = delta_t/2.0;
        A_continuity.insert(2*i + 1, 3*i + 3) = 0.0;
        A_continuity.insert(2*i + 1, 3*i + 4) = -1.0;
        A_continuity.insert(2*i + 1, 3*i + 5) = delta_t/2.0;
    }
    low_boundary_continuity.setZero(2*point_num -2);
    up_boundary_continuity.setZero(2*point_num -2);

    //2.2不允许倒车约束
    Eigen::SparseMatrix<double> A_avoid_reverse;
    Eigen::VectorXd low_boundary_avoid_reverse, up_boundary_avoid_reverse;

    A_avoid_reverse.resize(point_num - 1, 3*point_num);
    up_boundary_avoid_reverse.setZero(point_num-1);
    low_boundary_avoid_reverse.setZero(point_num-1);
    for (size_t i = 0; i < point_num - 1; i++)
    {
        A_avoid_reverse.insert(i, 3*i + 2) = 1;
        A_avoid_reverse.insert(i, 3*(i+1) + 2) = -1;
        low_boundary_avoid_reverse[i] = std::numeric_limits<double>::lowest();
    }
    
    //2.3凸空间约束
    Eigen::SparseMatrix<double> A_convex_space;
    Eigen::VectorXd low_boundary_convex_space, up_boundary_convex_space;

    A_convex_space.resize(2*point_num, 3*point_num);
    low_boundary_convex_space.setZero(2*point_num);
    up_boundary_convex_space.setZero(2*point_num);
    for (size_t i = 0; i < point_num; i++)
    {
        A_convex_space.insert(2*i + 0, 3*i + 0) = 1;
        A_convex_space.insert(2*i + 1, 3*i + 1) = 1;
        low_boundary_convex_space[2*i + 0] = s_lb[i];
        low_boundary_convex_space[2*i + 1] = s_dot_lb[i];
        up_boundary_convex_space[2*i + 0] = s_ub[i];
        up_boundary_convex_space[2*i + 1] = s_dot_ub[i];
    }

    //2.4组装矩阵
    Eigen::SparseMatrix<double> A_total, A_total_transpose;
    Eigen::VectorXd low_boundary_total, up_boundary_total;
    
    A_total_transpose.resize(3*point_num, 5*point_num-3);
    A_total_transpose.middleCols(0, A_continuity.rows()) = A_continuity.transpose();
    A_total_transpose.middleCols(A_continuity.rows(), A_avoid_reverse.rows()) = A_avoid_reverse.transpose();
    A_total_transpose.middleCols(A_continuity.rows()+A_avoid_reverse.rows(), A_convex_space.rows()) = A_convex_space.transpose();
    A_total = A_total_transpose.transpose();

    low_boundary_total.resize(5*point_num - 3);
    low_boundary_total << low_boundary_continuity, low_boundary_avoid_reverse, low_boundary_convex_space;
    up_boundary_total.resize(5*point_num - 3);
    up_boundary_total << up_boundary_continuity, up_boundary_avoid_reverse, up_boundary_convex_space;


    //-----------------求解-----------------------


    _speed_qp_solver.data()->setNumberOfVariables(3*point_num);
    _speed_qp_solver.data()->setNumberOfConstraints(A_total.rows());
    if(!_speed_qp_solver.data()->setHessianMatrix(H_total)) {return false;}
    if(!_speed_qp_solver.data()->setGradient(f)) {return false;}
    if(!_speed_qp_solver.data()->setLinearConstraintsMatrix(A_total)) {return false;}
    if(!_speed_qp_solver.data()->setLowerBound(low_boundary_total)) {return false;}
    if(!_speed_qp_solver.data()->setUpperBound(up_boundary_total)) {return false;}
    if(!_speed_qp_solver.initSolver()) {return false;}
    if(_speed_qp_solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {return false;}

    auto solution = _speed_qp_solver.getSolution();
    for (size_t i = 0; i < point_num; i++)
    {
        STPoint cur_st_point;
        cur_st_point.t = dp_speed_profile[i].t;
        cur_st_point.s = solution[3*i + 0];
        cur_st_point.s_dot = solution[3*i + 1];
        cur_st_point.s_dot_dot = solution[3*i + 2];

        qp_speed_profile.emplace_back(cur_st_point);
    }
    
    _speed_qp_solver.data()->clearHessianMatrix();
    _speed_qp_solver.data()->clearLinearConstraintsMatrix();
    _speed_qp_solver.clearSolverVariables();
    _speed_qp_solver.clearSolver();
    
    return true;
}

//加密二次规划所得速度剖面
void EMPlanner::increased_speed_profile(const std::vector<STPoint>& init_speed_profile, std::vector<STPoint>& final_speed_proflie)
{
    double dt = 0.02;//加密到每隔0.02s一个点
    for (size_t i = 0; i < init_speed_profile.size()-1; i++)
    {
        STPoint start_point(init_speed_profile[i]);
        double start_time = start_point.t;
        STPoint end_point(init_speed_profile[i+1]);
        size_t num = floor((end_point.t - start_point.t)/dt);
        PolynomialCurve curve;
        curve.curve_fitting(start_point.t, start_point.s, start_point.s_dot, start_point.s_dot_dot,
                            end_point.t, end_point.s, end_point.s_dot, end_point.s_dot_dot);
        for (size_t j = 0; j < num; j++)//包含起点不包含终点,避免终点重复包含
        {
            STPoint increased_point;
            double cur_t = start_time + j*dt;
            increased_point.t = cur_t;
            increased_point.s = curve.value_evaluation(cur_t, 0);
            increased_point.s_dot = curve.value_evaluation(cur_t, 1);
            increased_point.s_dot_dot = curve.value_evaluation(cur_t, 2);

            final_speed_proflie.emplace_back(increased_point);
        }   
    }
}

void EMPlanner::generate_trajectory(const std::vector<STPoint>& final_speed_profile,
                                    const std::vector<TrajectoryPoint>& path_trajectory,
                                    const std::vector<double>& path_index2s,
                                    const double planning_start_point_time_stamped ,
                                    std::vector<TrajectoryPoint>& trajectory)
{
    for (size_t i = 0; i < final_speed_profile.size(); i++)
    {
        using namespace std::chrono_literals;
        TrajectoryPoint trajectory_point;
        trajectory_point.v = final_speed_profile[i].s_dot;
        trajectory_point.a_tau = final_speed_profile[i].s_dot_dot;
        trajectory_point.time_stamped = planning_start_point_time_stamped + (final_speed_profile[i].t);

        double cur_s = final_speed_profile[i].s;
        double nearest_index;
        if (cur_s == path_index2s.back())
        {
            nearest_index = path_index2s.size()-1;
        }
        
        for (size_t j = 0; j < path_index2s.size()-1; j++)
        {
            if (cur_s >= path_index2s[j] && cur_s < path_index2s[j+1])
            {
                nearest_index = j;
            }
        }

        trajectory_point.x = path_trajectory[nearest_index].x +
         ((path_trajectory[nearest_index+1].x - path_trajectory[nearest_index].x)/(path_index2s[nearest_index+1]-path_index2s[nearest_index]))*(cur_s-path_index2s[nearest_index]);

        trajectory_point.y = path_trajectory[nearest_index].y +
         ((path_trajectory[nearest_index+1].y - path_trajectory[nearest_index].y)/(path_index2s[nearest_index+1]-path_index2s[nearest_index]))*(cur_s-path_index2s[nearest_index]);

        trajectory_point.heading = path_trajectory[nearest_index].heading +
         ((path_trajectory[nearest_index+1].heading - path_trajectory[nearest_index].heading)/(path_index2s[nearest_index+1]-path_index2s[nearest_index]))*(cur_s-path_index2s[nearest_index]);

        trajectory_point.kappa = path_trajectory[nearest_index].kappa +
         ((path_trajectory[nearest_index+1].kappa - path_trajectory[nearest_index].kappa)/(path_index2s[nearest_index+1]-path_index2s[nearest_index]))*(cur_s-path_index2s[nearest_index]);

        trajectory.emplace_back(trajectory_point);
         
    }
}