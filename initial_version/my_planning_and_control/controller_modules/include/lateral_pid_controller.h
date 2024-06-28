#pragma once

#include <unordered_map>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.h"
#include "rclcpp/rclcpp.hpp"
#include "common.h"


class LateralPIDController
{
public:
    LateralPIDController();

private:
    std::unordered_map<std::string,double> _args_lateral ;
    double _lateral_error_proportional;
    double _lateral_error_integral;
    double _lateral_error_derivative;
    double _lateral_previous_error__proportional;

public:
    double run_step( const geometry_msgs::msg::PoseStamped& target_pose, const VehicleState& current_ego_state);

};