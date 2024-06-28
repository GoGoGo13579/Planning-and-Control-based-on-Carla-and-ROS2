#include "lateral_pid_controller.h"

LateralPIDController::LateralPIDController()
{

    _args_lateral.emplace("K_P",1.0);
    _args_lateral.emplace("K_I",0.0);
    _args_lateral.emplace("K_D",0.0);


    _lateral_error_proportional = 0.0;
    _lateral_error_integral = 0.0;
    _lateral_error_derivative = 0.0;
    _lateral_previous_error__proportional = 0.0;
}

double LateralPIDController::run_step(const geometry_msgs::msg::PoseStamped& target_pose, const VehicleState& current_ego_state)
{
    double K_P = _args_lateral.at("K_P");
    double K_I = _args_lateral.at("K_I");
    double K_D = _args_lateral.at("K_D");
    double error_angle = 0.0;

    double yaw = current_ego_state.heading;
    //double yaw = q_host.getAngle();//注意范围0-2pi

    tf2::Vector3 v_host(std::cos(yaw),std::sin(yaw),0.0);
    tf2::Vector3 v_host_to_targer(target_pose.pose.position.x - current_ego_state.x,
                                    target_pose.pose.position.y - current_ego_state.y,
                                    0.0);
    // RCLCPP_INFO(rclcpp::get_logger("lateral_pid_controller"),"主车位置(%.2f,%.2f),目标点位置(%.2f,%.2f),叉乘值%.2f",current_ego_state.x,current_ego_state.y,
    // target_pose.pose.position.x,target_pose.pose.position.y,tf2::tf2Cross(v_host,v_host_to_targer).getZ());
    if(tf2::tf2Cross(v_host,v_host_to_targer).getZ()>1e-3)
    {//表示v_host在v_targer_to_host的右侧，要左打轮，steer为负值
        error_angle = -tf2::tf2Angle(v_host,v_host_to_targer);
    }
    else if(tf2::tf2Cross(v_host,v_host_to_targer).getZ() < -1e-3) 
    {
        error_angle = tf2::tf2Angle(v_host,v_host_to_targer);
    }
    else 
    {
        error_angle = 0.0;
    }

    _lateral_previous_error__proportional  = _lateral_error_proportional;
    _lateral_error_proportional = error_angle;
    _lateral_error_integral = std::min(std::max(_lateral_error_integral + error_angle,-400.0),400.0);
    _lateral_error_derivative = error_angle - _lateral_previous_error__proportional;

    double steer = std::min(std::max(K_P*_lateral_error_proportional + K_I*_lateral_error_integral + K_D*_lateral_error_derivative,-1.0),1.0);
    // RCLCPP_INFO(rclcpp::get_logger("lateral_pid_controller"),"横向PID数据:PID参数(%.2f,%.2f,%.2f),各项误差(%.2f,%.2f,%.2f),角度误差%.2f,控制指令%.2f"
    // ,K_P,K_I,K_D,_lateral_error_proportional,_lateral_error_integral,_lateral_error_derivative,error_angle,steer);

    return steer;
}