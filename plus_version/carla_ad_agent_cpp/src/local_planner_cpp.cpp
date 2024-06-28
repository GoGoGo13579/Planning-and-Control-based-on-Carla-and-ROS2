#include "local_planner.h"


class VehiclePIDCotroller
{
public:
    VehiclePIDCotroller(const std::unordered_map<std::string,double>& args_lateral={},
                        const std::unordered_map<std::string,double>& args_longitudinal={})
    {

        
        if (!args_lateral.empty()){
            _args_lateral = args_lateral;
        }else{
            _args_lateral.emplace("K_P",1.0);
            _args_lateral.emplace("K_I",0.0);
            _args_lateral.emplace("K_D",0.0);
        }

        if (!args_longitudinal.empty()){
            _args_longitudinal = args_longitudinal;
        }else{
            _args_longitudinal.emplace("K_P",1.0);
            _args_longitudinal.emplace("K_I",0.0);
            _args_longitudinal.emplace("K_D",0.0);
        }

        _lateral_error_proportional = 0.0;
        _lateral_error_integral = 0.0;
        _lateral_error_derivative = 0.0;
        _lateral_previous_error__proportional = 0.0;

        _longitudinal_error_proportional=0.0;
        _longitudinal_error_integral=0.0;
        _longitudinal_error_derivative=0.0;
        _longitudinal_previous_error__proportional = 0.0;

    }

private:
    std::unordered_map<std::string,double> _args_lateral ;
    std::unordered_map<std::string,double> _args_longitudinal;
    //将误差声明成类变量，提高声明周期
    double _lateral_error_proportional;
    double _lateral_error_integral;
    double _lateral_error_derivative;
    double _lateral_previous_error__proportional;

    double _longitudinal_error_proportional;
    double _longitudinal_error_integral;
    double _longitudinal_error_derivative;
    double _longitudinal_previous_error__proportional;


public:
    carla_msgs::msg::CarlaEgoVehicleControl run_step(const double& target_speed, const double& current_speed,
    const geometry_msgs::msg::PoseStamped& target_pose, const geometry_msgs::msg::Pose& current_pose )
    {

        carla_msgs::msg::CarlaEgoVehicleControl control_msg;
        control_msg.steer = lateral_pid_controller(target_pose, current_pose,_args_lateral);
        control_msg.throttle = longitudianl_pid_controller(target_speed, current_speed,_args_longitudinal);
        control_msg.brake = 0.0;
        control_msg.hand_brake = false;
        control_msg.manual_gear_shift = false;
        return control_msg;
    }

private:
    double longitudianl_pid_controller(const double& target_speed, const double& current_speed,const std::unordered_map<std::string,double>& args_lateral)
    {   
        //有const只能用at,因为对于[],在没有对应键的情况下会插入一个新值
        double K_P = args_lateral.at("K_P");
        double K_I = args_lateral.at("K_I");
        double K_D = args_lateral.at("K_D");
        _lateral_previous_error__proportional = _lateral_error_proportional;
        _lateral_error_proportional = target_speed - current_speed;
        _lateral_error_integral = std::min(std::max(_lateral_error_integral+_lateral_error_proportional,-40.0),40.0);
        _lateral_error_derivative = _lateral_error_proportional - _lateral_previous_error__proportional;

        double throttle = std::min(std::max(K_P * _lateral_error_proportional + K_I * _lateral_error_integral + K_D * _lateral_error_derivative,0.0),1.0);

        //这个控制器连刹车的功能都没有，只能加速

        RCLCPP_INFO(rclcpp::get_logger("lateral_pid_controller"),"纵向PID数据:PID参数(%.2f,%.2f,%.2f),各项误差(%.2f,%.2f,%.2f),期望速度%.2f,实际速度%.2f,控制指令%.2f"
        ,K_P,K_I,K_D,_lateral_error_proportional,_lateral_error_integral,_lateral_error_derivative,target_speed,current_speed,throttle);


        return throttle;


    }

    double lateral_pid_controller( const geometry_msgs::msg::PoseStamped& target_pose,
     const geometry_msgs::msg::Pose& current_pose,const std::unordered_map<std::string,double> args_longitudinal)
    {
        double K_P = args_longitudinal.at("K_P");
        double K_I = args_longitudinal.at("K_I");
        double K_D = args_longitudinal.at("K_D");
        double error_angle = 0.0;

        tf2::Quaternion q_host(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
        double yaw = q_host.getAngle();//注意范围0-2pi

        tf2::Vector3 v_host(std::cos(yaw),std::sin(yaw),0.0);
        tf2::Vector3 v_host_to_targer(target_pose.pose.position.x - current_pose.position.x,
                                      target_pose.pose.position.y - current_pose.position.y,
                                      0.0);
        RCLCPP_INFO(rclcpp::get_logger("lateral_pid_controller"),"主车位置(%.2f,%.2f),目标点位置(%.2f,%.2f),叉乘值%.2f",current_pose.position.x,current_pose.position.y,
        target_pose.pose.position.x,target_pose.pose.position.y,tf2::tf2Cross(v_host,v_host_to_targer).getZ());
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

        _longitudinal_previous_error__proportional  = _longitudinal_error_proportional;
        _longitudinal_error_proportional = error_angle;
        _longitudinal_error_integral = std::min(std::max(_longitudinal_error_integral + error_angle,-400.0),400.0);
        _longitudinal_error_derivative = error_angle - _longitudinal_previous_error__proportional;

        double steer = std::min(std::max(K_P*_longitudinal_error_proportional + K_I*_longitudinal_error_integral + K_D*_longitudinal_error_derivative,-1.0),1.0);
        RCLCPP_INFO(rclcpp::get_logger("lateral_pid_controller"),"横向PID数据:PID参数(%.2f,%.2f,%.2f),各项误差(%.2f,%.2f,%.2f),角度误差%.2f,控制指令%.2f"
        ,K_P,K_I,K_D,_longitudinal_error_proportional,_longitudinal_error_integral,_longitudinal_error_derivative,error_angle,steer);

        return steer;
        
    }

};

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner():Node("local_planner"),MIN_DISTANCE_PERCENTAGE(0.35),_buffer_size(5)
    {
        RCLCPP_INFO(this->get_logger(),"local_planner_node创建！");
        //获取主车名称
        if (has_parameter("role_name")){
            role_name = this->get_parameter("role_name").as_string();
        }
        else{
            role_name = "ego_vehicle";
        }
        //横向PId参数
        if (has_parameter("Kp_lateral")){
            args_lateral_unordered_map["K_P"] = get_parameter("Kp_lateral").as_double();
        }
        else{
            args_lateral_unordered_map["K_P"] = 1.2;
        }

        if (has_parameter("Ki_lateral")){
            args_lateral_unordered_map["K_I"] = get_parameter("Ki_lateral").as_double();
        }
        else{
            args_lateral_unordered_map["K_I"] = 0.0;
        }

        if (has_parameter("Kd_lateral")){
            args_lateral_unordered_map["K_D"] = get_parameter("Kd_lateral").as_double();
        }
        else{
            args_lateral_unordered_map["K_D"] = 0.3;
        }

        //纵向PID参数
        if (has_parameter("Kp_longitudinal")){
            args_longitudinal_unordered_map["K_P"] = get_parameter("Kp_longitudinal").as_double();
        }
        else{
            args_longitudinal_unordered_map["K_P"] = 0.206;
        }

        if (has_parameter("Ki_longitudinal")){
            args_longitudinal_unordered_map["K_I"] = get_parameter("Ki_longitudinal").as_double();
        }
        else{
            args_longitudinal_unordered_map["K_I"] = 0.0206;
        }

        if (has_parameter("Kd_longitudinal")){
            args_longitudinal_unordered_map["K_D"] = get_parameter("Kd_longitudinal").as_double();
        }
        else{
            args_longitudinal_unordered_map["K_D"] = 0.515;
        }

        //控制步长
        if (has_parameter("control_time_step")){
            _control_time_step = get_parameter("control_time_step").as_double();
        }
        else{
             _control_time_step = 0.05;
        }

        //订阅方
        //创建里程计订阅方，订阅车辆当前位姿消息
        _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            //话题名称
            "/carla/" + role_name +"/odometry",
            10,
            std::bind(&LocalPlanner::odometry_cb,this,std::placeholders::_1)
        );
        
        //创建期望速度订阅方
        _target_speed_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "/carla/" + role_name +"/speed_command",
            10,
            std::bind(&LocalPlanner::target_speed_cb,this,std::placeholders::_1)
        );

        //创建路径订阅方
        _path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
            "/carla/" + role_name + "/waypoints",
            10,
            std::bind(&LocalPlanner::path_cb,this,std::placeholders::_1)
        );


        //发布方
        _control_cmd_publisher = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            "/carla/" + role_name + "/vehicle_control_cmd",
            10
        );

        _targer_pose_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
            "/carla/" + role_name + "/next_target",
            10
        );

        //创建时钟，按照控制时间步来执行PID控制
        _timer = this->create_wall_timer(std::chrono::milliseconds(int(_control_time_step*1000)),std::bind(&LocalPlanner::run_step,this));

        //初始化控制器
        _vehicle_controller = std::make_shared<VehiclePIDCotroller>(args_lateral_unordered_map,args_longitudinal_unordered_map);
    }

private:
    //参数
    double MIN_DISTANCE_PERCENTAGE ;
    std::string role_name;
    std::unordered_map<std::string,double> args_lateral_unordered_map;
    std::unordered_map<std::string,double> args_longitudinal_unordered_map;

    //订阅方以及订阅的数据
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;//里程计订阅方，订阅本车当前位姿与速度
    geometry_msgs::msg::Pose _current_pose;//本车当前位置
    double _current_speed;//本车当前速度

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _target_speed_subscriber;
    double _target_speed;//期望速度,单位为km/h

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscriber;//路径订阅方
    int _buffer_size;//路径点缓存大小
    std::deque<geometry_msgs::msg::PoseStamped> _path_buffer;//路径点缓存容器
    std::deque<geometry_msgs::msg::PoseStamped> _path_deque;//整条路径存储容器

    //发布方
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr _control_cmd_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _targer_pose_publisher;

    //定时器与控制器
    std::shared_ptr<VehiclePIDCotroller> _vehicle_controller;//一定要在这里就定义出来，不然误差存储不下来
    rclcpp::TimerBase::SharedPtr _timer;
    double _control_time_step;


    //里程计订阅方回调函数，获取当前本车位姿与速度
    void odometry_cb(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _current_pose = msg->pose.pose;
        _current_speed = std::sqrt(std::pow(msg->twist.twist.linear.x,2) 
                                +  std::pow(msg->twist.twist.linear.y,2)
                                +  std::pow(msg->twist.twist.linear.y,2))*3.6;
        //里程计的单位是m/s，转化为km/h
    }

    //速度指令订阅方回调函数，获取有ad_agent发布的速度指令
    void target_speed_cb(std_msgs::msg::Float64::SharedPtr msg)
    {
        _target_speed = msg->data;
    }

    void path_cb(nav_msgs::msg::Path::SharedPtr waypoints)
    {
        _path_buffer.clear();
        _path_deque.clear();
        for (auto &&pose : waypoints->poses)
        {
            //RCLCPP_INFO(this->get_logger(),"目标位置(%.2f,%.2f)",pose.pose.position.x,pose.pose.position.y);
            _path_deque.push_back(pose);
        }
        //第一个点丢掉，初始点再车后头
        //_path_deque.pop_front();
        
    }

    //单步执行PID控制
    void run_step()
    {
        //没收到路径信息
        if((_path_buffer.empty())&&(_path_deque.empty()))
        {
            RCLCPP_INFO(this->get_logger(),"等待路径信息......");
            emergency_stop();
            return;
        }
        //目标速度为0
        if(_target_speed == 0.0)
        {
            emergency_stop();
            return;
        }
        //将路径点缓存一下,其实这里有一个问题，可能会把_path_deque里面取空
        if(_path_buffer.empty())
        {
            // for (auto &&pose : _path_deque)
            // {
            //     RCLCPP_INFO(this->get_logger(),"缓存前_path_deque(%.2f,%.2f)",pose.pose.position.x,pose.pose.position.y);
            // }
            
            for(int i=0 ; i<_buffer_size ; i++)
            {
                _path_buffer.push_back(_path_deque[0]);
                _path_deque.pop_front();
            }
            // for (auto &&pose : _path_deque)
            // {
            //     RCLCPP_INFO(this->get_logger(),"缓存后_path_deque(%.2f,%.2f)",pose.pose.position.x,pose.pose.position.y);
            // }
            for (auto &&pose : _path_buffer)
            {
                RCLCPP_INFO(this->get_logger(),"缓存后_path_buffer(%.2f,%.2f)",pose.pose.position.x,pose.pose.position.y);
            }
        }

        auto target_pose = _path_buffer[0];
        //RCLCPP_INFO(this->get_logger(),"目标位置(%.2f,%.2f)",target_pose.pose.position.x,target_pose.pose.position.y);
        //****发布标记
        visualization_msgs::msg::Marker marker;
        marker.type = 0;
        marker.header.frame_id = "map";
        marker.pose = target_pose.pose;
        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 255;
        marker.color.a = 1.0;
        _targer_pose_publisher->publish(marker);
        

        //计算并发布控制指令
        auto control_msg = this->_vehicle_controller->run_step(_target_speed,_current_speed,target_pose,_current_pose);
        _control_cmd_publisher->publish(control_msg);

        //清除落后的路径点,这里感觉有问腿，应该根据方向来去除点
        int max_index = -1;

        double sampling_radius = _target_speed / 3.6;
        double min_distance = sampling_radius * MIN_DISTANCE_PERCENTAGE;
        double distance;
        for(int i = 0 ; i<(int)_path_buffer.size() ; i++)//这里不能一次取5个，取两次就空了
        {
            distance = std::hypot(_current_pose.position.x - _path_buffer[i].pose.position.x,
                                  _current_pose.position.y - _path_buffer[i].pose.position.y) ;
            if (distance < min_distance)
            {
                max_index = i;
            }
        }
        if (max_index >= 0)
        {
            for (int i = 0; i <= max_index; i++)
            {
                _path_buffer.pop_front();
            } 
        }
    }


public:

    //发布紧急制动指令
    void emergency_stop()
    {
        carla_msgs::msg::CarlaEgoVehicleControl control_msg ;
        control_msg.throttle = 0.0;
        control_msg.steer = 0.0;
        control_msg.brake = 1.0;
        control_msg.hand_brake = false;
        control_msg.manual_gear_shift = false;
        control_msg.reverse = false;

        _control_cmd_publisher->publish(control_msg);

    }

};



int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);

    auto local_planner = std::make_shared<LocalPlanner>();

    rclcpp::on_shutdown(std::bind(&LocalPlanner::emergency_stop,local_planner));

    rclcpp::spin(local_planner);

    rclcpp::shutdown();
    return 0;
}
