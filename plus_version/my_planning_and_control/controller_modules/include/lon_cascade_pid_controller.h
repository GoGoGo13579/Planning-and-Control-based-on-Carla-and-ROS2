#pragma once
#include <vector>
#include <memory>
#include "pid_controller.h"
#include "common.h"
#include "rclcpp/rclcpp.hpp"

class LonCascadePIDController
{
public:
    //构造函数,初始化默认控制器参数
    LonCascadePIDController();

    //计算控制指令
    bool compute_control_cmd(const std::vector<TrajectoryPoint>& trajectory,
                             const std::shared_ptr<VehicleState>& ego_state,
                             const double cur_t, const double dt,
                             ControlCMD & cmd);

    //计算目标点
    void compute_target_point(const std::vector<TrajectoryPoint>& trajectory,
                              const double predicted_time,
                              TrajectoryPoint& target_point, double& target_point_s);

    //设置位置PID参数
    void set_station_controller(const double k_p, const double k_i, const double k_d);

    //设置速度PID参数
    void set_speed_controller(const double k_p, const double k_i, const double k_d);

    //设置位置PID积分限幅边界
    void set_station_integral_saturation_boundary(
        const double high_boundary,
        const double low_boundary
    );

    //设置速度PID积分限幅边界
    void set_speed_integral_saturation_boundary(
        const double high_boundary,
        const double low_boundary
    );
    
    //重置积分项
    void reset_integral();

private:
    std::unique_ptr<PIDController> _station_controller;//位置PID
    std::unique_ptr<PIDController> _speed_controller;//速度PID

    double _preview_window = 5;//预瞄点步长
};