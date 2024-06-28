#include "emplanner.h"

auto LOG = rclcpp::get_logger("emplanner");

EMPlanner::EMPlanner() : Node("emlpanner")
{
    _get_waypoint_client = this->create_client<carla_waypoint_types::srv::GetWaypoint>(
    "/carla_waypoint_publisher/ego_vehicle/get_waypoint"
    );
    // _reference_speed_subscriber = this->create_subscription<std_msgs::msg::Float64>(
    //     "/carla/ego_vehicle/speed_command",
    //     10,
    //     std::bind(&EMPlanner::reference_speed_callback,this,std::placeholders::_1)
    // );

    //路径决策代价初始化
    _weight_coefficients.path_dp_w_ref = 200.0;
    _weight_coefficients.path_dp_w_dl = 300.0;
    _weight_coefficients.path_dp_w_ddl = 200.0;
    _weight_coefficients.path_dp_w_dddl = 1000.0;
    _weight_coefficients.path_dp_w_obs = 1e5;

    //路径规划代价初始化
    _weight_coefficients.path_qp_w_ref = 100.0;
    _weight_coefficients.path_qp_w_dl = 500.0;
    _weight_coefficients.path_qp_w_ddl = 50.0;
    _weight_coefficients.path_qp_w_dddl = 10.0;
    _weight_coefficients.path_qp_w_mid = 20.0;
    _weight_coefficients.path_qp_w_l_end = 40.0;
    _weight_coefficients.path_qp_w_dl_end = 40.0;
    _weight_coefficients.path_qp_w_ddl_end = 40.0;

    //速度决策代价初始化
    _weight_coefficients.speed_dp_w_ref_speed = 1000.0;
    _weight_coefficients.speed_dp_w_a = 300.0;
    _weight_coefficients.speed_dp_w_jerk = 300.0;
    _weight_coefficients.speed_dp_w_obs = 1e5;

    //速度规划代价初始化
    _weight_coefficients.speed_qp_w_ref_speed = 200.0;
    _weight_coefficients.speed_qp_w_a = 100.0;
    _weight_coefficients.speed_qp_w_jerk = 100.0;

    //绘图
    _path_polt_handle = matplot::figure();
    _trajectory_plot_handle = matplot::figure();
    _plot_count = 0;

    //路径二次规划
    _path_qp_solver.settings()->setWarmStart(true);
    _speed_qp_solver.settings()->setWarmStart(true);

    _reference_speed = 8.0;
    

}


// void EMPlanner::reference_speed_callback(std_msgs::msg::Float64::SharedPtr msg)
// {
//     //  _reference_speed = msg->data/3.6; //km/h转化为m/s
//     //  RCLCPP_INFO(this->get_logger(),"参考速度为:",_reference_speed);
// }

void EMPlanner::planning_run_step(const std::vector<PathPoint> reference_line,const std::shared_ptr<VehicleState> ego_state, 
                        const std::vector<derived_object_msgs::msg::Object>& obstacles,  std::vector<TrajectoryPoint>& final_trajectory)
{   _current_time = this->now().seconds();
    //1障碍物处理
    //1.1区别动态与静态障碍物，存储在类成员变量里
    obstacle_fileter(ego_state, obstacles);
    //2.2障碍物坐标转换
    auto reference_index2s = calculate_index_to_s(reference_line,ego_state);
    // for (auto &&s : reference_index2s)
    // {
    //     RCLCPP_INFO(LOG,"index2s表%.3f",s);
    // }
    
    std::vector<FrenetPoint> static_obstacle_frenet_coordinate;

    if(!_static_obstacles.empty())
    {

        cartesion_set_to_frenet_set(_static_obstacles, reference_line, ego_state, static_obstacle_frenet_coordinate);

        for (auto && s_obs_frenet : static_obstacle_frenet_coordinate)
        {
            RCLCPP_INFO(LOG,"静态障碍物信息(%.3f,%.3f)",s_obs_frenet.s,s_obs_frenet.l);
        }
        
        
    }

    //2.确定规划起点并投影到frenet坐标系
    TrajectoryPoint planning_start_point = calculate_planning_start_point(ego_state);
    std::vector<FrenetPoint> temp_set;
    cartesion_set_to_frenet_set(planning_start_point, reference_line, ego_state, temp_set);
    FrenetPoint planning_start_point_frenet_coordinate = temp_set.front();
    planning_start_point_frenet_coordinate.l_prime_prime = 0.0;


    //-----------------------------------3.路径规划--------------------------------------
    //3.1获取车道宽度
    double lane_width = 4.0;
    //double car_width = 2.0;

    //这部分注释是调用服务通信获取路宽
    // auto get_waypoint_request = std::make_shared<carla_waypoint_types::srv::GetWaypoint::Request>();
    // get_waypoint_request->location.x = ego_state->x;
    // get_waypoint_request->location.y = ego_state->y;
    // get_waypoint_request->location.z = ego_state->z;
    // auto response = _get_waypoint_client->async_send_request(get_waypoint_request);
    // if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),response) == rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     lane_width = response.get()->waypoint.lane_width;

    //         RCLCPP_INFO(this->get_logger(),"请求成功,车道宽度为:%.3f",lane_width);
    // }

    //3.2dp路径规划
    //3.2.1获取采用点
    std::vector<std::vector<FrenetPoint>> path_dp_sample;
    get_path_dp_sample(path_dp_sample,planning_start_point_frenet_coordinate,10.0,6,1.0,7);

    // for (size_t level = 1; level <= path_dp_sample.size() ; level++)
    // {
    //     RCLCPP_INFO(this->get_logger(),"-------第%d列采样点--------",level);
    //     auto vector_point = path_dp_sample[level-1];
    //     for (auto &&point : vector_point)
    //     {
    //         RCLCPP_INFO(this->get_logger(),"采样点坐标(%.3f,%.3f)",point.s,point.l);
    //     }   
    // }

    //3.2.2获取最小代价
    std::vector<std::vector<DpPathNode>> path_dp_node_table;
    path_dp_node_table.emplace_back();
    path_dp_node_table.back().emplace_back(planning_start_point_frenet_coordinate,0.0,nullptr);
    auto path_dp_sample_front = path_dp_sample.front().front();
    for (size_t level = 1; level < path_dp_sample.size(); level++)
    {
        path_dp_node_table.emplace_back();
        if (level == 1)//第一列
        {   
            //遍历第一列每一个节点，与起始点计算代价，算出来的代价直接是最小代价
            for (auto && current_sample_point : path_dp_sample[level])
            {
                path_dp_node_table.back().emplace_back(
                    current_sample_point,
                    calculate_dp_cost(path_dp_sample_front, current_sample_point, static_obstacle_frenet_coordinate, _weight_coefficients),
                    std::make_shared<DpPathNode>(path_dp_node_table.front().front()));
            }
        }
        else
        {
            //对于不是第一列的点，要对该列的每一个点与前一列的每一个计算代价
            for (auto && current_sample_point : path_dp_sample[level])
            {
                double min_cost = std::numeric_limits<double>::max();
                size_t index;
                for (size_t i = 0 ; i < path_dp_node_table[level -1].size(); i++)
                {
                    //当前代价 = 前一列点的最小代价 + 前一列点到当前点的代价
                    double cur_cost =
                    path_dp_node_table[level-1][i].min_cost +
                    calculate_dp_cost(path_dp_node_table[level-1][i].sl_point, current_sample_point, static_obstacle_frenet_coordinate, _weight_coefficients);
                    if ( cur_cost < min_cost )
                    {
                        //记录最小代价与相应的前一列点的索引
                        min_cost = cur_cost;
                        index = i;
                    }              
                }
                path_dp_node_table.back().emplace_back(
                    current_sample_point,
                    min_cost,
                    std::make_shared<DpPathNode>(path_dp_node_table[level-1][index])
                );
            }
        }
    }

    //3.2.3利用path_dp_node_table找出最优路径
    size_t path_min_cost_index;
    double path_min_cost = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_dp_node_table.back().size(); i++)
    {
        if (path_dp_node_table.back()[i].min_cost < path_min_cost)
        {
            path_min_cost = path_dp_node_table.back()[i].min_cost;
            path_min_cost_index = i;
        }
    }
    auto path_useful_node = path_dp_node_table.back()[path_min_cost_index];//最后一列的最小代价节点
    
    std::vector<FrenetPoint> dp_best_path;//存放dp产生的最优路径
    dp_best_path.emplace_back(path_useful_node.sl_point);
    while (true)
    {
        //由最后一列的最小代价节点中的指针，一步一步向前推，把整条路径找出来给
        path_useful_node = *(path_useful_node.pre_node);
        dp_best_path.emplace(dp_best_path.begin(),path_useful_node.sl_point);
        if(dp_best_path.front().s == planning_start_point_frenet_coordinate.s &&
           dp_best_path.front().l == planning_start_point_frenet_coordinate.l)
        {
            break;
        }
    }
    RCLCPP_INFO(this->get_logger(),"emplanner错误定位1");
    //3.2.4对动态规划出来的路径进行加密
    std::vector<FrenetPoint> final_dp_path;
    increased_dp_path(dp_best_path, 1.0, final_dp_path);

    //3.2.5获取凸空间
    //现在假定静态障碍物都是车，一般化情况应该是根据相应的障碍物类型获得相应的边框
    std::vector<double> final_dp_path_l_max;
    std::vector<double> final_dp_path_l_min;
    double safe_distance = 0.0;
    double road_up_boundary = 1.5 * lane_width  - safe_distance;
    double road_low_boundary = -(0.5 * lane_width  - safe_distance);
    generate_convex_space(final_dp_path, road_up_boundary, road_low_boundary, static_obstacle_frenet_coordinate, final_dp_path_l_max, final_dp_path_l_min);

    RCLCPP_INFO(this->get_logger(),"emplanner错误定位2");
    //3.3qp路径规划
    std::vector<FrenetPoint> init_qp_path;
    double l_desire = 0.0;
    double dl_desire = 0.0;
    double ddl_desire = 0.0;
    path_QP_planning(final_dp_path, final_dp_path_l_min, final_dp_path_l_max, l_desire, dl_desire, ddl_desire, _weight_coefficients, init_qp_path);

    //将路径转化为轨迹
    auto dp_trajectory = frenet_to_cartesion(init_qp_path, reference_line, reference_index2s);


    //-----------------------------------4.速度规划--------------------------------------
    
    //4.1路径规划结果的index_to_s表
    std::vector<double> path_index2s;
    path_index2s.emplace_back(0.0);
    for (size_t i = 1; i < dp_trajectory.size(); i++)
    {
        path_index2s.emplace_back(path_index2s.back() + std::hypot(dp_trajectory[i].x - dp_trajectory[i-1].x,
                                                                  dp_trajectory[i].y - dp_trajectory[i-1].y));
    }

    //4.2动态障碍物投影
    std::vector<FrenetPoint> dynamic_obstacle_frenet_coordinate;
    if(!_dynamic_obstacles.empty())
    {
        cartesion_set_to_frenet_set(_dynamic_obstacles, dp_trajectory, dp_trajectory.front(),dynamic_obstacle_frenet_coordinate);
    }
    //---------------测试
    //为了验证速度规划，手动添加几个动态障碍物
    // FrenetPoint test1, test2;
    // test1.s = 20;
    // test1.l = 4;
    // test1.s_dot = 1;
    // test1.l_dot = -1;
    // test2.s = 8;
    // test2.l = 4;
    // test2.s_dot = 1;
    // test2.l_dot = -2;
    // dynamic_obstacle_frenet_coordinate.emplace_back(test1);
    // dynamic_obstacle_frenet_coordinate.emplace_back(test2);
    // _reference_speed = 8;
    //4.3生成st图
    std::vector<std::unordered_map<std::string, double>> dynamic_obs_st_graph;
    if (!dynamic_obstacle_frenet_coordinate.empty())
    {
        generate_st_graph(dynamic_obstacle_frenet_coordinate, 2.0, dynamic_obs_st_graph); 
    }

    //4.4计算规划起点信息
    STPoint planning_start_point_st_coordinate = calculate_speed_start_point(planning_start_point);

    //4.5速度动态规划
    //4.5.1获取采样点
    std::vector<std::vector<STPoint>> speed_dp_sample;
    get_speed_dp_sample(planning_start_point_st_coordinate, 0.5, 8.0, path_index2s, speed_dp_sample);
    //4.5.2计算代价
    std::vector<std::vector<DpSpeedNode>> speed_dp_node_table;
    speed_dp_node_table.emplace_back();
    auto speed_dp_sample_front = speed_dp_sample.front().front();
    speed_dp_node_table.back().emplace_back(speed_dp_sample_front,0.0,nullptr);
    for (size_t level = 1; level < speed_dp_sample.size(); level++)
    {
        if (level == 1)
        {
            speed_dp_node_table.emplace_back();
            for (auto &&sample_point : speed_dp_sample[level])
            {
                STPoint pre_st_point(speed_dp_sample_front);
                STPoint cur_st_point(sample_point);
                double delat_t = cur_st_point.t - pre_st_point.t;
                cur_st_point.s_dot = (cur_st_point.s - pre_st_point.s)/delat_t;
                cur_st_point.s_dot_dot = (cur_st_point.s_dot - pre_st_point.s_dot)/delat_t;

                speed_dp_node_table.back().emplace_back(
                    cur_st_point,
                    calculate_speed_dp_cost(speed_dp_sample_front, sample_point, _reference_speed, dynamic_obs_st_graph, _weight_coefficients),
                    std::make_shared<DpSpeedNode>(speed_dp_node_table.front().front())
                );
            }
        }
        else
        {
            speed_dp_node_table.emplace_back();
            for (int i = 0; i < static_cast<int>(speed_dp_sample[level].size()); i++)
            {
                double min_speed_dp_cost = std::numeric_limits<double>::max();
                int pre_index = -1;
                for (int j = 0; j < (int)speed_dp_sample[level-1].size(); j++)
                {
                    double cur_cost = calculate_speed_dp_cost(speed_dp_node_table[level-1][j].st_point, speed_dp_sample[level][i],_reference_speed,dynamic_obs_st_graph,_weight_coefficients)
                                    + speed_dp_node_table[level-1][j].min_cost; //注意，这里面的start_point不能用speed_dp_sample里的点，因为那里面的没有赋值
                    if(cur_cost < min_speed_dp_cost)
                    {
                        min_speed_dp_cost = cur_cost;
                        pre_index = j;
                    }  
                }
                STPoint pre_st_point(speed_dp_node_table[level-1][pre_index].st_point);
                STPoint cur_st_point(speed_dp_sample[level][i]);
                double delat_t = cur_st_point.t - pre_st_point.t;
                cur_st_point.s_dot = (cur_st_point.s - pre_st_point.s)/delat_t;
                cur_st_point.s_dot_dot = (cur_st_point.s_dot - pre_st_point.s_dot)/delat_t;

                speed_dp_node_table.back().emplace_back(
                    cur_st_point,
                    min_speed_dp_cost,
                    std::make_shared<DpSpeedNode>(speed_dp_node_table[level-1][pre_index])
                );
            }
        }    
    }
    //4.5.3回溯出轨迹
    double trajectory_min_cost = std::numeric_limits<double>::max();
    int trajectory_min_cost_level = -1;
    int trajectory_min_cost_index = -1;
    for (int level = 1; level < (int)speed_dp_node_table.size(); level++)
    {   //搜索上边界
        if (speed_dp_node_table[level].front().min_cost < trajectory_min_cost)
        {
            trajectory_min_cost = speed_dp_node_table[level].front().min_cost;
            trajectory_min_cost_level = level;
            trajectory_min_cost_index = 0;   
        }
    }
    for (int index = 0; index < (int)speed_dp_node_table.back().size(); index++)
    {   //搜索右边界 
        if (speed_dp_node_table.back()[index].min_cost < trajectory_min_cost)
        {
            trajectory_min_cost = speed_dp_node_table.back()[index].min_cost;
            trajectory_min_cost_level = (int)speed_dp_node_table.size()-1;
            trajectory_min_cost_index = index;
        }
    }
    DpSpeedNode trajectory_useful_node(speed_dp_node_table[trajectory_min_cost_level][trajectory_min_cost_index]);
    std::deque<STPoint> dp_speed_profile;
    dp_speed_profile.emplace_front(trajectory_useful_node.st_point);
    while (true)
    {
        trajectory_useful_node = *trajectory_useful_node.pre_node;
        dp_speed_profile.emplace_front(trajectory_useful_node.st_point);
        if (trajectory_useful_node.st_point == speed_dp_sample_front)
        {
            break;
        }
    }
    RCLCPP_INFO(this->get_logger(),"emplanner错误定位3");
    //4.6速度二次规划
    //4.6.1生成凸空间
    std::vector<double> s_lb, s_ub, s_dot_lb, s_dot_ub;
    generate_convex_space(dp_trajectory, path_index2s, dp_speed_profile, dynamic_obs_st_graph, 0.2*9.8, s_lb, s_ub, s_dot_lb, s_dot_ub);
    
    //4.6.2二次规划
    std::vector<STPoint> init_qp_speed_profile;
    speed_QP_planning(dp_speed_profile, s_lb, s_ub, _reference_speed, s_dot_lb, s_dot_ub, _weight_coefficients, init_qp_speed_profile);

    //4.6.3加密二次规划得到的速度曲线
    std::vector<STPoint> final_qp_speed_profile;
    increased_speed_profile(init_qp_speed_profile, final_qp_speed_profile);

    //-----------------------------生成最终的轨迹----------------------------------

    //5.1将路径规划结果与速度规划结构拼成轨迹
    std::vector<TrajectoryPoint> init_trajectory;
    generate_trajectory(final_qp_speed_profile, dp_trajectory, path_index2s, planning_start_point.time_stamped, init_trajectory);

    //5.2轨迹拼接
    final_trajectory.clear();//清空原有轨迹
    if (!_switch_trajectory.empty())
    {
        for (auto &&trajectory_point : _switch_trajectory)
        {
            final_trajectory.emplace_back(trajectory_point);
        }
    }
    for (auto &&trajectory_point : init_trajectory)
    {
        final_trajectory.emplace_back(trajectory_point);
    }
    
    RCLCPP_INFO(this->get_logger(),"emplanner错误定位4");
    
    //5.3上一周期轨迹赋值
    _previous_trajectory.clear();
    for (auto &&trajectory_point : final_trajectory)
    {
        _previous_trajectory.emplace_back(trajectory_point);
    }
    
    

    //------------------------------------------------绘图------------------------------------------------
    if (_plot_count % 20 == 0)
    {
        matplot::figure(_path_polt_handle);
        matplot::cla();
        std::vector<double> init_dp_path_s,init_dp_path_l;
        std::vector<double> final_dp_path_s,final_dp_path_l;
        std::vector<double> init_qp_path_s, init_qp_path_l;

        for (size_t i = 0; i < dp_best_path.size(); i++)
        {
            init_dp_path_s.emplace_back(dp_best_path[i].s);    
            init_dp_path_l.emplace_back(dp_best_path[i].l);    
        }
        
        for (size_t i = 0; i < final_dp_path.size(); i++)
        {
            final_dp_path_s.push_back(final_dp_path[i].s);
            final_dp_path_l.push_back(final_dp_path[i].l);
            init_qp_path_s.emplace_back(init_qp_path[i].s);
            init_qp_path_l.emplace_back(init_qp_path[i].l);
        }
        //dp加密前
        //matplot::plot(init_dp_path_s, init_dp_path_l, "bs-");
        //dp加密后曲线
        matplot::plot(final_dp_path_s, final_dp_path_l,"bo-")->line_width(2);
        matplot::hold(true);
        //凸空间
        matplot::plot(final_dp_path_s, final_dp_path_l_min, "r*-")->line_width(2);
        matplot::plot(final_dp_path_s, final_dp_path_l_max, "ro-")->line_width(2);
        //qp规划曲线
        matplot::plot(init_qp_path_s, init_qp_path_l, "go-")->line_width(2);
        //绘制静态障碍物
        for (auto &&static_obs_sl_point : static_obstacle_frenet_coordinate)
        {
            matplot::line(static_obs_sl_point.s - 2.5, static_obs_sl_point.l + 1, static_obs_sl_point.s + 2.5, static_obs_sl_point.l + 1)->line_width(2);
            matplot::line(static_obs_sl_point.s + 2.5, static_obs_sl_point.l + 1, static_obs_sl_point.s + 2.5, static_obs_sl_point.l - 1)->line_width(2);
            matplot::line(static_obs_sl_point.s + 2.5, static_obs_sl_point.l - 1, static_obs_sl_point.s - 2.5, static_obs_sl_point.l - 1)->line_width(2);
            matplot::line(static_obs_sl_point.s - 2.5, static_obs_sl_point.l - 1, static_obs_sl_point.s - 2.5, static_obs_sl_point.l + 1)->line_width(2) ;
        }
        //matplot::legend({"DP planning ","up doundary","low boundary","QP planning"});

    }
    if (_plot_count % 20 == 0)
    {
        matplot::figure(_trajectory_plot_handle);
        matplot::cla();
        //动态规划轨迹
        std::vector<double> init_t_set, init_s_set;
        matplot::hold(true);
        for (auto &&st_point : dp_speed_profile)
        {
            init_t_set.emplace_back(st_point.t);
            init_s_set.emplace_back(st_point.s);
        }
        matplot::plot(init_t_set, init_s_set, "bo-")->line_width(2);

        //二次规划轨迹
        std::vector<double> init_qp_speed_profile_t_set, init_qp_speed_profile_s_set;
        for (auto &&st_point : init_qp_speed_profile)
        {
            init_qp_speed_profile_t_set.emplace_back(st_point.t);
            init_qp_speed_profile_s_set.emplace_back(st_point.s);
        }
        matplot::plot(init_qp_speed_profile_t_set, init_qp_speed_profile_s_set, "g*-")->line_width(2);
        
        
        //动态障碍物st图
        for (auto &&st_graph_node : dynamic_obs_st_graph)
        {
            matplot::line(st_graph_node.at("t_in"),st_graph_node.at("s_in"),
                          st_graph_node.at("t_out"), st_graph_node.at("s_out"))->line_width(2);
        }
        
        
    }
    
    _plot_count++;
    if (_plot_count == 1e10)
    {
        _plot_count = 0;
    }
 
    RCLCPP_INFO(this->get_logger(),"轨迹包含点数:%d",final_trajectory.size());
    RCLCPP_INFO(this->get_logger(),"emplanner错误定位5");

}

//获取规划起点与拼接轨迹
TrajectoryPoint EMPlanner::calculate_planning_start_point(std::shared_ptr<VehicleState> ego_state)
{
    _switch_trajectory.clear();
    TrajectoryPoint planning_start_point;
    //如果第一次运行
    double delta_T = 0.1;
    if(_previous_trajectory.empty())
    {
        planning_start_point.x = ego_state->x;
        planning_start_point.y = ego_state->y;
        planning_start_point.v = 0.0;
        planning_start_point.heading = ego_state->heading;
        planning_start_point.ax = 0.0;
        planning_start_point.ay = 0.0;
        planning_start_point.kappa = 0.0;
        planning_start_point.time_stamped = _current_time + delta_T;
    }
    else//不是第一次运行，已经有了上一周期规划的轨迹
    {
        //计算主车位置与目标点之间的误差
        size_t current_time_index = -1;
        if (_current_time <= _previous_trajectory[0].time_stamped)
        {
            current_time_index  = 0;
        }
        for (size_t i = 0; i < _previous_trajectory.size()-2; i++)//获取当前时刻在上一周期轨迹对应的时间索引
        {
            if (_current_time > _previous_trajectory[i].time_stamped && _current_time <= _previous_trajectory[i+1].time_stamped)
            {
                if((_current_time - _previous_trajectory[i].time_stamped)<=(_previous_trajectory[i+1].time_stamped - _current_time))
                {
                    current_time_index  = i;
                }
                else
                {
                    current_time_index = i+1;
                }
            }
        }
        auto target_point = _previous_trajectory[current_time_index];
        Eigen::Vector2d tau_target_point(std::cos(target_point.heading), std::sin(target_point.heading));
        Eigen::Vector2d nor_target_point(-std::sin(target_point.heading), std::cos(target_point.heading));
        Eigen::Vector2d host_to_target_point(target_point.x - ego_state->x, target_point.y - ego_state->y);
        double error_longitudional = std::abs(host_to_target_point.dot(tau_target_point));//纵向误差
        double error_lateral = std::abs(host_to_target_point.dot(nor_target_point));//横向误差

        if(error_lateral < 0.5 && error_longitudional < 1.5)//主车实际位置与目标点差距不大
        {
            //在上一周期轨迹上搜索规划起点
            size_t start_time_index = -1;
            double start_time = _current_time + delta_T;
            if(start_time <= _previous_trajectory[0].time_stamped)
            {
                start_time_index = 0;
            }
            for (size_t i = 0; i < _previous_trajectory.size()-2; i++)
            {
                if (start_time > _previous_trajectory[i].time_stamped && start_time <= _previous_trajectory[i+1].time_stamped)
                {
                    if((start_time - _previous_trajectory[i].time_stamped)<=(_previous_trajectory[i+1].time_stamped - start_time))
                    {
                        start_time_index  = i;
                    }
                    else
                    {
                        start_time_index = i+1;
                    }
                }
            }

            planning_start_point.x = _previous_trajectory[start_time_index].x;
            planning_start_point.y = _previous_trajectory[start_time_index].y;
            planning_start_point.v = _previous_trajectory[start_time_index].v;
            planning_start_point.heading = _previous_trajectory[start_time_index].heading;
            planning_start_point.ax = _previous_trajectory[start_time_index].ax;
            planning_start_point.ay = _previous_trajectory[start_time_index].ay;
            //这里还要一个值得商讨的问题，实际使用current_time+delta_T还是上一周期上start_time_index所对应轨迹点的时间
            planning_start_point.time_stamped = start_time;

            //这个时候还要拼接上一段轨迹,向前拼20个点
            if(start_time_index >= 20)
            {
                for (int i = start_time_index - 1; i >= 0 && (start_time_index-i)<=20 ; i--)
                {
                    _switch_trajectory.emplace_front(_previous_trajectory[i]);//这个排列顺序是序号越大，时间越靠后的
                }
            }
            else if(start_time_index > 0)
            {
                for (int i = start_time_index - 1; i >= 0; i--)
                {
                    _switch_trajectory.emplace_front(_previous_trajectory[i]);//这个排列顺序是序号越大，时间越靠后的 
                }
            }
        }
        else//主车实际位置与目标点差距很大
        {
            //用动力学方程外推,认为在着100ms中加速度变化很小
            planning_start_point.ax = ego_state->ax;
            planning_start_point.ay = ego_state->ay;
            double ego_vx = ego_state->v * std::cos(ego_state->heading);
            double ego_vy = ego_state->v * std::sin(ego_state->heading);
            double planning_start_point_vx = ego_vx + ego_state->ax * delta_T;
            double planning_start_point_vy = ego_vy + ego_state->ay * delta_T;
            planning_start_point.v = std::hypot(planning_start_point_vx,planning_start_point_vy);
            planning_start_point.heading = std::atan2(planning_start_point_vy,planning_start_point_vx);
            planning_start_point.x = ego_state->x + ego_vx*delta_T + 0.5*ego_state->ax*delta_T*delta_T;
            planning_start_point.y = ego_state->y + ego_vy*delta_T + 0.5*ego_state->ay*delta_T*delta_T;
            planning_start_point.time_stamped = _current_time + delta_T;
        }
        

    }

    return planning_start_point;
}


void EMPlanner::obstacle_fileter(std::shared_ptr<VehicleState> ego_state, const std::vector<derived_object_msgs::msg::Object>& obstacles)
{
    auto LOG = rclcpp::get_logger("emplanner");
    //将障碍物分成动态与静态障碍物
    _static_obstacles.clear();
    _dynamic_obstacles.clear();
    if(obstacles.empty())
    {
        return;
    }
    for (auto &&obs : obstacles)
    {
        if(obs.id == ego_state->id){continue;}//障碍物不包含主车

        double v_obs = std::sqrt(std::pow(obs.twist.linear.x,2.0)+std::pow(obs.twist.linear.y,2.0)+std::pow(obs.twist.linear.z,2.0));//障碍物速度
        Eigen::Vector2d host_to_obs(obs.pose.position.x - ego_state->x, obs.pose.position.y - ego_state->y);//主车到障碍物的向量
        Eigen::Vector2d tau_host(std::cos(ego_state->heading), std::sin(ego_state->heading));
        Eigen::Vector2d nor_host(-std::sin(ego_state->heading), std::cos(ego_state->heading));
        double longitudinal_d = host_to_obs.dot(tau_host);//纵向距离
        double lateral_d = host_to_obs.dot(nor_host);//横向距离
        if(v_obs < 1e-2)//静态障碍物，即使有加速度，一个规划周期是0.1s，障碍物以最大加速度加速也达不到很大的速度
        {
            if(longitudinal_d <= 60 && longitudinal_d >= -10 && lateral_d <= 10 && lateral_d >= -10)
            {
                _static_obstacles.push_back(obs);
            }
            else
            {
                continue;
            }
        }
        else//动态障碍物 
        {
            if(longitudinal_d <= 60 && longitudinal_d >= -10 && lateral_d <= 20 && lateral_d >= -20)
            {
                _dynamic_obstacles.push_back(obs);
            }
            else
            {
                continue;
            }
        }
    }
}

