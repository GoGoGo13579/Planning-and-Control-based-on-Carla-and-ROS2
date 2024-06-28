#include "lon_cascade_pid_controller.h"

//构造函数,初始化默认控制器参数
LonCascadePIDController::LonCascadePIDController()
{
    _station_controller = std::make_unique<PIDController>();
    _speed_controller = std::make_unique<PIDController>();
    //设置控制器参数
    _station_controller->set_controller(1.0, 0.0, 0.0);
    _speed_controller->set_controller(4.0, 0.0, 0.0);
}

//计算控制指令
bool LonCascadePIDController::compute_control_cmd
    (const std::vector<TrajectoryPoint>& trajectory,
     const std::shared_ptr<VehicleState>& ego_state,
     const double cur_t, const double dt,
     ControlCMD & cmd)
{
    auto LOG = rclcpp::get_logger("lon_pid");

    //1.计算横向误差
    //1.1生成index2s表

    //1.2计算投影点
    std::vector<FrenetPoint> temp_sl_set;
    cartesion_set_to_frenet_set(ego_state, trajectory, temp_sl_set);
    FrenetPoint ego_pro(std::move(temp_sl_set.front()));

    //2.计算目标点
    TrajectoryPoint target_point;
    double target_point_s; //目标点的s坐标 
    double predicted_time = cur_t +  _preview_window*dt;
    compute_target_point(trajectory, predicted_time, target_point, target_point_s);

    //3.计算控制量
    //3.1位置PID
    double station_error = target_point_s - ego_pro.s;//位置误差
    double speed_offset;//位置PID的输出
    _station_controller->computer_control_cmd(station_error, dt, speed_offset);

    //3.2速度PID
    double speed_error = target_point.v - ego_pro.s_dot;
    double speed_controller_input = speed_error + speed_offset;
    double acceleration_cmd;
    _speed_controller->computer_control_cmd(speed_controller_input, dt, acceleration_cmd);

    //缺少油门刹车表的转换
    if (acceleration_cmd >= 0)
    {
        cmd.brake = 0.0;
        cmd.throttle = std::max(std::min(acceleration_cmd, 1.0),0.0);
    }
    else
    {
        cmd.throttle = 0.0;
        //乘这个0.1是为了避免刹车太硬,改善控制效果
        cmd.brake = std::max(std::min(-acceleration_cmd * 0.1, 1.0),0.0);
    }
    
    RCLCPP_INFO(LOG, "当前位置:%.3f, 期望位置:%.3f, 位置控制器输出:%.3f",ego_pro.s, target_point_s, speed_offset);
    RCLCPP_INFO(LOG, "当前速度:%.3f, 期望速度:%.3f, 速度控制器输出:%.3f",ego_pro.s_dot,target_point.v, acceleration_cmd);
    RCLCPP_INFO(LOG, "油门指令:%.3f, 刹车指令:%.3f",cmd.throttle, cmd.brake);

    return true;
}

//计算目标点位置
void LonCascadePIDController::compute_target_point(
                              const std::vector<TrajectoryPoint>& trajectory,
                              const double predicted_time,
                              TrajectoryPoint& target_point, double& target_point_s)
{
    //这里更好的处理方式是在TrajectoryPoint里加入s成员变量
    //但是那样做要修改的地方太多了所以还是使用index2s的方法
    std::vector<double> index2s;
    index2s.emplace_back(0.0);
    for (size_t i = 1; i < trajectory.size(); ++i)
    {
        index2s.emplace_back(
            std::hypot(trajectory[i].x - trajectory[i-1].x,
                       trajectory[i].y - trajectory[i-1].y)
            + index2s[i-1]
        );
    }

    int index_nearest = -1;
    for (int i = 0; i < static_cast<int>(trajectory.size())-1; ++i)
    {
        if (predicted_time >= trajectory[i].time_stamped && predicted_time < trajectory[i+1].time_stamped)
        {
            index_nearest = i;
        }
    }
    if (index_nearest == -1)
    {
        return ;
    }

    double delta_t = (trajectory.at(index_nearest+1).time_stamped - trajectory.at(index_nearest).time_stamped);
    double dt = predicted_time - trajectory.at(index_nearest).time_stamped;

    double k_x = (trajectory.at(index_nearest+1).x - trajectory.at(index_nearest).x)/delta_t;
    target_point.x = trajectory.at(index_nearest).x + k_x*dt;

    double k_y = (trajectory.at(index_nearest+1).y - trajectory.at(index_nearest).y)/delta_t;
    target_point.y = trajectory.at(index_nearest).y + k_y*dt;

    double k_v = (trajectory.at(index_nearest+1).v - trajectory.at(index_nearest).v)/delta_t;
    target_point.v = trajectory.at(index_nearest).v + k_v*dt;

    double k_heading = (trajectory.at(index_nearest+1).heading - trajectory.at(index_nearest).heading)/delta_t;
    target_point.heading = trajectory.at(index_nearest).heading + k_heading*dt;

    double k_a_tau = (trajectory.at(index_nearest+1).a_tau - trajectory.at(index_nearest).a_tau)/delta_t;    
    target_point.a_tau = trajectory.at(index_nearest).a_tau + k_a_tau*dt; 

    double k_s = (index2s[index_nearest+1] - index2s[index_nearest])/delta_t;
    target_point_s = index2s[index_nearest] + k_s*dt;
}


//设置位置PID参数
void LonCascadePIDController::set_station_controller
    (const double k_p, const double k_i, const double k_d)
{
    _station_controller->set_controller(k_p, k_i, k_d);
}

//设置速度PID参数
void LonCascadePIDController::set_speed_controller
     (const double k_p, const double k_i, const double k_d)
{
    _speed_controller->set_controller(k_p, k_i, k_d);
}

//设置位置PID积分限幅边界
void LonCascadePIDController::set_station_integral_saturation_boundary
    (const double high_boundary,const double low_boundary)
{
    _station_controller->set_integral_saturation_boundary(high_boundary, low_boundary);
}

//设置速度PID积分限幅边界
void LonCascadePIDController::set_speed_integral_saturation_boundary
     (const double high_boundary,const double low_boundary)
{
    _speed_controller->set_integral_saturation_boundary(high_boundary, low_boundary);
}

//重置积分项
void LonCascadePIDController::reset_integral()
{
    _station_controller->reset_integral();
    _speed_controller->reset_integral();
}

