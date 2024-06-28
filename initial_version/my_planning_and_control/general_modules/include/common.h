#pragma once
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include "derived_object_msgs/msg/object.hpp"

#include "tf2_eigen/tf2_eigen.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

struct TrajectoryPoint
{
    TrajectoryPoint() : x(0.0), y(0.0), heading(0.0), kappa(0.0), v(0.0),
                        ax(0.0), ay(0.0), a_tau(0.0), time_stamped(0.0){};
    // TrajectoryPoint(const TrajectoryPoint& other);
    //TrajectoryPoint& operator=(const TrajectoryPoint& other);
    double x;
    double y;
    double heading;
    double kappa;
    double v;
    double ax;
    double ay;
    double a_tau;//切向加速度
    double time_stamped;
};

struct PathPoint
{
    PathPoint() : x(0.0), y(0.0), heading(0.0), kappa(0.0){};
    double x;
    double y;
    double heading;
    double kappa;
};

struct FrenetPoint
{
    FrenetPoint() : s(0.0), l(0.0), s_dot(0.0), l_dot(0.0), l_prime(0.0),
                    s_dot_dot(0.0), l_dot_dot(0.0), l_prime_prime(0.0){};
    double s;
    double l;
    double s_dot;
    double l_dot;
    double l_prime;
    double s_dot_dot;
    double l_dot_dot;
    double l_prime_prime;
};

struct STPoint
{
    STPoint() : t(0.0), s(0.0), s_dot(0.0), s_dot_dot(0.0){};
    double t;
    double s;
    double s_dot;
    double s_dot_dot;

    bool operator == (const STPoint& other)
    {
        return (t == other.t)&&(s == other.s)&&(s_dot == other.s_dot)&&(s_dot_dot == other.s_dot_dot);
    }
};

struct VehicleState
{
    VehicleState() : x(0.0), y(0.0), z(0.0), heading(0.0), v(0.0), ax(0.0), ay(0.0), 
                     omega(0.0), alpha(0.0), time_stamp(0.0), id(0), 
                     flag_imu(false), flag_ode(false), flag_info(false){};
    double x;
    double y;
    double z;
    double heading;
    double v;
    double ax;
    double ay;
    double omega;
    double alpha;
    double time_stamp;
    uint32_t id;
    bool flag_imu;
    bool flag_ode;
    bool flag_info;
};


//计算路径的偏航角与曲率
void Calculate_heading_and_kappa(std::shared_ptr<std::vector<PathPoint>> path);
//计算投影点
void calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const Eigen::Vector2d point, int& match_point_index, PathPoint& projection_point);
void calculate_projection_point(std::shared_ptr<std::vector<PathPoint>> path, const TrajectoryPoint& trajectory_point, int& match_point_index, PathPoint& match_point, PathPoint& projection_point);//重载版本

//计算index2s表
std::vector<double> calculate_index_to_s(std::shared_ptr<std::vector<PathPoint>> path, std::shared_ptr<VehicleState> vehicle_state);

//笛卡尔坐标到frenet坐标的转换
FrenetPoint cartesion_to_frenet(const TrajectoryPoint& host, const PathPoint& projection_point, const PathPoint& match_point, std::vector<double> index2s, const int& match_point_index);
void cartesion_set_to_frenet_set(const std::vector<derived_object_msgs::msg::Object>& object_set, const std::vector<TrajectoryPoint>& trajectory, const TrajectoryPoint& original_point, std::vector<FrenetPoint>& frenet_set);
void cartesion_set_to_frenet_set(const std::vector<derived_object_msgs::msg::Object>& object_set, const std::vector<PathPoint>& path, std::shared_ptr<VehicleState> vehicle_state, std::vector<FrenetPoint>& frenet_set);
void cartesion_set_to_frenet_set(const TrajectoryPoint& trajectory_point, const std::vector<PathPoint>& path, std::shared_ptr<VehicleState> vehicle_state, std::vector<FrenetPoint>& frenet_set);
//frenet坐标系到笛卡尔坐标系的转变
std::vector<TrajectoryPoint> frenet_to_cartesion(const std::vector<FrenetPoint>& frenet_point_set, const std::shared_ptr<std::vector<PathPoint>> cartesian_path,
                                                 const std::vector<double> cartesian_path_index2s);

//object转换为轨迹点
TrajectoryPoint object_to_trajectory_point(const derived_object_msgs::msg::Object object);
//车辆状态转化为轨迹点
TrajectoryPoint vehicle_state_to_trajectory_point(const std::shared_ptr<VehicleState> vehicle_state);