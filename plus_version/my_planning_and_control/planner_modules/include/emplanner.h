#pragma once
#include <iostream>
#include <vector>
#include <deque>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "common.h"
#include "polynomial_curve.h"
#include "derived_object_msgs/msg/object.hpp"
#include "carla_waypoint_types/srv/get_waypoint.hpp"
#include "std_msgs/msg/float64.hpp"
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include "matplot/matplot.h"


struct DpPathNode
{
    DpPathNode();
    DpPathNode(const FrenetPoint& args_sl_point, const double args_min_cost, const std::shared_ptr<DpPathNode> args_pre_node);
    
    FrenetPoint sl_point;
    double min_cost;
    std::shared_ptr<DpPathNode> pre_node;
};

struct DpSpeedNode
{
    DpSpeedNode();
    DpSpeedNode(const STPoint& args_st_point, const double args_min_cost, const std::shared_ptr<DpSpeedNode> args_pre_node);
    STPoint st_point;
    double min_cost;
    std::shared_ptr<DpSpeedNode> pre_node;
};

struct WeightCoefficients
{
    //路径动态规划权重系数
    double path_dp_w_ref;
    double path_dp_w_dl;
    double path_dp_w_ddl;
    double path_dp_w_dddl;
    double path_dp_w_obs;

    //路径二次规划权重系数
    double path_qp_w_ref;
    double path_qp_w_dl;
    double path_qp_w_ddl;
    double path_qp_w_dddl;
    double path_qp_w_mid;
    double path_qp_w_l_end;
    double path_qp_w_dl_end;
    double path_qp_w_ddl_end;

    //速度动态规划权重系数
    double speed_dp_w_ref_speed;
    double speed_dp_w_a;
    double speed_dp_w_jerk;
    double speed_dp_w_obs;

    //速度二次规划权重系数
    double speed_qp_w_ref_speed;
    double speed_qp_w_a;
    double speed_qp_w_jerk;
};


class EMPlanner : public rclcpp::Node
{
public:
    EMPlanner();
private:
    std::vector<derived_object_msgs::msg::Object> _static_obstacles;//静态障碍物
    std::vector<derived_object_msgs::msg::Object> _dynamic_obstacles;//动态障碍物
    std::deque<TrajectoryPoint> _previous_trajectory;//上一周期的轨迹
    std::deque<TrajectoryPoint> _switch_trajectory;//拼接轨迹

    rclcpp::Client<carla_waypoint_types::srv::GetWaypoint>::SharedPtr _get_waypoint_client;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _reference_speed_subscriber;
    //void reference_speed_callback(std_msgs::msg::Float64::SharedPtr msg);
    double _reference_speed;
    double _current_time;

    WeightCoefficients _weight_coefficients;//路径，速度规划的权重系数

    //绘图
    matplot::figure_handle _path_polt_handle;
    size_t _plot_count;
    matplot::figure_handle _trajectory_plot_handle;
public:
    //------------empalnner主流程--------------------
    //在emplanner.cpp源文件里
    void planning_run_step(const std::vector<PathPoint> reference_line,const std::shared_ptr<VehicleState> ego_state, 
                           const std::vector<derived_object_msgs::msg::Object>& obstacles, std::vector<TrajectoryPoint>& trajectory);

private:
    //区分静态与动态障碍物
    void obstacle_fileter(std::shared_ptr<VehicleState> ego_state, const std::vector<derived_object_msgs::msg::Object>& obstacles);

    //确定规划起点
    TrajectoryPoint calculate_planning_start_point(std::shared_ptr<VehicleState> ego_state);


    //----------------------路径规划--------------------------
    //在emplanner_path_planning.cpp源文件里
    //获取路径规划的dp采样点
    OsqpEigen::Solver _path_qp_solver;//路径二次规划求解器

    void get_path_dp_sample(std::vector<std::vector<FrenetPoint>>& path_dp_sample, const FrenetPoint& planning_start_sl, 
                        const double s_sample_distance, const size_t s_sample_number, 
                        const double l_sample_distance, const size_t l_sample_number);

    //计算路径规划中动态规划的代价
    double calculate_dp_cost(const FrenetPoint& start_point, const FrenetPoint& end_point, const std::vector<FrenetPoint>& static_obstacles_sl_set,
                             const WeightCoefficients& weight_coefficients);
    
    //加密动态规划的结果
    void increased_dp_path(const std::vector<FrenetPoint>& init_dp_path, const double increased_distance, std::vector<FrenetPoint>& final_dp_path);

    //生成凸空间
    void generate_convex_space(const std::vector<FrenetPoint>& final_dp_path, const double road_up_boundary, const double road_low_boundary,
                               const std::vector<FrenetPoint>& static_obs_sl_point_set, std::vector<double>& path_l_max, std::vector<double>& path_l_min);

    //在路径上寻找与障碍物s坐标的离的最近的（这里的近指的是s坐标的距离最小）路径点索引
    int find_index_for_obs_on_path(const std::vector<FrenetPoint>& final_dp_path, const double static_obs_s);

    //路径QP规划
    bool path_QP_planning(const std::vector<FrenetPoint>& final_dp_path, const std::vector<double>& final_dp_path_lmin, const std::vector<double>& final_dp_path_lmax, 
                          const double l_desire, const double dl_desire, const double ddl_desire,
                          const WeightCoefficients& weight_coeficients, std::vector<FrenetPoint>& init_qp_path );


    //-----------------------速度规划--------------------------
    //在emplanner_speed_planning.cpp文件里

    OsqpEigen::Solver _speed_qp_solver;

    //生成st图
    void generate_st_graph(const std::vector<FrenetPoint>& dynamic_obs_sl_set, const double delat_l, std::vector<std::unordered_map<std::string, double>>& st_graph);

    //计算速度规划的起点
    STPoint calculate_speed_start_point(const TrajectoryPoint& planning_start_point);

    //获取采样点
    void get_speed_dp_sample(const STPoint& planning_start_point_st_coordinate, const double t_sample_distance, const double t_end, const std::vector<double>& index2s,std::vector<std::vector<STPoint>>& speed_dp_sample);

    //计算速度规划中动态规划的代价
    double calculate_speed_dp_cost(const STPoint& start_point, const STPoint& end_point, const double reference_speed,
                                   const std::vector<std::unordered_map<std::string, double>>& dynamic_obs_st_graph, const WeightCoefficients& weight_coefficients);

    //生成凸空间
    void generate_convex_space(const std::vector<TrajectoryPoint>& init_trajectory, const std::vector<double> path_index2s, const std::deque<STPoint>& speed_profile, const std::vector<std::unordered_map<std::string, double>>& dynamic_obs_st_graph, 
                           const  double& ay_max, std::vector<double>& s_lb, std::vector<double>& s_ub, std::vector<double>& s_dot_lb, std::vector<double>& s_dot_ub);
    
    //在dp速度剖面上上搜索obs的对应的索引
    int find_t_index(const std::deque<STPoint>& speed_profile, const double t);

    //速度二次规划
    bool speed_QP_planning(const std::deque<STPoint>& dp_speed_profile, const std::vector<double>& s_lb, const std::vector<double>& s_ub, const double reference_speed, 
                           const std::vector<double>& s_dot_lb, const std::vector<double>& s_dot_ub, const WeightCoefficients& weight_coeficients, std::vector<STPoint>& qp_speed_profile);

    //加密二次规划所得速度剖面
    void increased_speed_profile(const std::vector<STPoint>& init_speed_profile, std::vector<STPoint>& final_speed_proflie);

    //将路径规划结果于速度规划结果组装成一条轨迹
    void generate_trajectory(const std::vector<STPoint>& final_speed_profile, const std::vector<TrajectoryPoint>& path_trajectory, const std::vector<double>& path_index2s,
                                    const double planning_start_point_time_stamped ,std::vector<TrajectoryPoint>& trajecotry);
};