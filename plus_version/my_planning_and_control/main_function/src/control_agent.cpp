#include"control_agent.h"

class ControlAgent : public rclcpp::Node
//控制代理,用于实现产生控制指令
{
public:
    ControlAgent() : Node("control_agent")
    {
        _role_name = "ego_vehicle";

        //订阅方
        //里程计信息订阅方
        _ego_state = std::make_shared<VehicleState>();
        _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/carla/" + _role_name +"/odometry",
            10,
            std::bind(&ControlAgent::odometry_cb,this,std::placeholders::_1) 
        );
        //创建惯性导航订阅方，订阅车辆当前加速度和角速度消息
        _imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            "/carla/" + _role_name +"/imu",
            10,
            std::bind(&ControlAgent::imu_cb,this,std::placeholders::_1)
        );
        //创建车辆信息订阅方，订阅车辆id号
        _ego_info_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleInfo>(
            "/carla/ego_vehicle/vehicle_info",
            10,
            std::bind(&ControlAgent::ego_info_cb,this,std::placeholders::_1)
        );
        _trajectory_subscriber = this->create_subscription<pnc_msgs::msg::Trajectory>(
             "/carla/" + _role_name + "/planning_trajectory",
             10,
             std::bind(&ControlAgent::trajectory_cb,this,std::placeholders::_1)
        );


        //发布方
        _control_cmd_publisher = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            "/carla/" + _role_name + "/vehicle_control_cmd",
            10
        );

        //控制器
        _control_time_step = 0.02;//20ms执行一次控制
        _control_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(_control_time_step*1000)),
                                                 std::bind(&ControlAgent::control_run_step,this));
        _lon_cascade_pid_controller = std::make_unique<LonCascadePIDController>();
        _lateral_lqr_controller = std::make_unique<LateralLQRController>();


    }
private:
    std::string _role_name ;//主车名称

    //订阅方以及订阅的数据
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;//里程计订阅方，订阅本车当前位姿与速度
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscriber;//惯性导航订阅方，订阅加速度与角速度
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr _ego_info_subscriber;//定于车辆的车道信息
    std::shared_ptr<VehicleState> _ego_state; //主车信息

    rclcpp::Subscription<pnc_msgs::msg::Trajectory>::SharedPtr _trajectory_subscriber;//规划轨迹订阅方
    std::vector<TrajectoryPoint> _trajectory;//规划轨迹

    //发布方
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr _control_cmd_publisher;//控制指令发布方

    //控制器
    std::unique_ptr<LonCascadePIDController> _lon_cascade_pid_controller;//纵向位置PID控制器
    std::unique_ptr<LateralLQRController> _lateral_lqr_controller;//横向lqr控制器
    rclcpp::TimerBase::SharedPtr _control_timer;
    double _control_time_step;
    
public:
    void control_run_step()
    {

        //1.信息检查
        //1.1车辆状态信息检查
        if (_ego_state->flag_imu == false ||
            _ego_state->flag_info == false ||
            _ego_state->flag_ode == false)
        {
            emergency_stop();
            RCLCPP_ERROR(this->get_logger(),"车辆信息缺失,等待车辆信息！！！");
            return;
        }
        
        //1.2轨迹信息检查
            
        if (_trajectory.empty())
        {
            emergency_stop();
            RCLCPP_ERROR(this->get_logger(),"规划轨迹信息缺失,等待规划轨迹信息！！！");
            return;
        }

        
        //2.求解控制指令
        double cur_t = this->now().seconds();
        ControlCMD cmd;
        //2.1纵向控制
        _lon_cascade_pid_controller->set_station_controller(0.1, 0.0, 0.0);
        _lon_cascade_pid_controller->set_speed_controller(2.0, 0.2, 0.0);
        _lon_cascade_pid_controller->set_speed_integral_saturation_boundary(0.3, -0.3);
        _lon_cascade_pid_controller->compute_control_cmd(_trajectory, _ego_state, cur_t, _control_time_step, cmd);
        //2.2横向控制
        _lateral_lqr_controller->set_r_matrix(1.0);
        std::vector<double> q_vector = {0.05, 0.0, 1.0, 0.0};
        _lateral_lqr_controller->set_q_matrix(q_vector);
        _lateral_lqr_controller->compute_control_cmd(_trajectory, _ego_state, cur_t, _control_time_step, cmd);

        //3.发布控制指令
        carla_msgs::msg::CarlaEgoVehicleControl cmd_msg;
        cmd_msg.brake = cmd.brake;
        cmd_msg.steer = cmd.steer;
        cmd_msg.throttle = cmd.throttle;
        cmd_msg.hand_brake = false;
        cmd_msg.manual_gear_shift = false;
        _control_cmd_publisher->publish(cmd_msg);
    }

private:

    void odometry_cb(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _ego_state->x = msg->pose.pose.position.x;
        _ego_state->y = msg->pose.pose.position.y;
        _ego_state->z = msg->pose.pose.position.z;
        _ego_state->v = std::sqrt(std::pow(msg->twist.twist.linear.x,2) 
                                +  std::pow(msg->twist.twist.linear.y,2)
                                +  std::pow(msg->twist.twist.linear.z,2));
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg->pose.pose.orientation,tf_q);
        double roll,pitch,yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll,pitch,yaw);
        _ego_state->heading = tf2NormalizeAngle(yaw);
        _ego_state->flag_ode = true;
        //里程计的单位是m/s，转化为km/h
    }

    void imu_cb(sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {

        _ego_state->ax = imu_msg->linear_acceleration.x;
        _ego_state->ay = imu_msg->linear_acceleration.y;
        _ego_state->flag_imu = true;
    }

    void ego_info_cb(carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr ego_info)
    {
        _ego_state->id = ego_info->id;
        _ego_state->flag_info = true;
    }

    void trajectory_cb(pnc_msgs::msg::Trajectory::SharedPtr trajectory)
    {
        _trajectory.clear();
        TrajectoryPoint sub_point; 
        for (auto &point : trajectory->points)
        {
            sub_point.x = point.x;
            sub_point.y = point.y;
            sub_point.heading = point.heading;
            sub_point.kappa = point.kappa;
            sub_point.v = point.v;
            sub_point.ax = point.ax;
            sub_point.ay = point.ay;
            sub_point.a_tau = point.a_tau;
            sub_point.time_stamped = point.time_stamped;

            _trajectory.emplace_back(sub_point);
        }
    }

    void emergency_stop()
    {
        //紧急停车指令
        carla_msgs::msg::CarlaEgoVehicleControl control_msg;
        control_msg.brake = 1.0;
        control_msg.throttle = 0.0;
        control_msg.steer = 0.0;
        control_msg.reverse = false;
        _control_cmd_publisher->publish(control_msg);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ControlAgent>());

    rclcpp::shutdown();

    return 0;
}
