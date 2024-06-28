#include "main_function.h"

class MyPlanningANDControl : public rclcpp::Node
{
public:
  MyPlanningANDControl() : Node("my_planning_and_control"),MIN_DISTANCE_PERCENTAGE(0.5)
  {
    //主车名字

    role_name = "ego_vehicle";


    //控制器时钟，按照控制时间步来执行控制
    _control_time_step = 0.02;//10ms执行一次控制
    _control_timer = this->create_wall_timer(std::chrono::milliseconds(int(_control_time_step*1000)),std::bind(&MyPlanningANDControl::control_run_step,this));
    _longitudinal_pid_controller = std::make_shared<LongitudinalPIDController>();
    _lateral_pid_controller = std::make_shared<LateralPIDController>();
    _lateral_lqr_controller = std::make_shared<LaterLQRController>();
    //规划时钟
    _planning_time_step = 0.2;//100ms执行一次规划
    _reference_line_generator = std::make_shared<ReferenceLine>();//参考线生成器
    _reference_line = std::make_shared<std::vector<PathPoint>>();
    _planning_timer = this->create_wall_timer(std::chrono::milliseconds(int(_planning_time_step*1000)),std::bind(&MyPlanningANDControl::planning_run_step,this));
    _emplanner = std::make_shared<EMPlanner>();

    //订阅方
    //创建里程计订阅方，订阅车辆当前位姿消息
    _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        //话题名称
        "/carla/" + role_name +"/odometry",
        10,
        std::bind(&MyPlanningANDControl::odometry_cb,this,std::placeholders::_1)
    );
    //创建惯性导航订阅方，订阅车辆当前加速度和角速度消息
    _imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
      "/carla/" + role_name +"/imu",
      10,
      std::bind(&MyPlanningANDControl::imu_cb,this,std::placeholders::_1)
    );
    //创建车辆信息订阅方，订阅车辆id号
    _ego_info_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleInfo>(
      "/carla/ego_vehicle/vehicle_info",
      10,
      std::bind(&MyPlanningANDControl::ego_info_cb,this,std::placeholders::_1)
    );
    
    //创建期望速度订阅方
    _target_speed_subscriber = this->create_subscription<std_msgs::msg::Float64>(
        "/carla/" + role_name +"/speed_command",
        10,
        std::bind(&MyPlanningANDControl::target_speed_cb,this,std::placeholders::_1)
    );

    //创建路径订阅方
    _global_path = std::make_shared<std::vector<PathPoint>>();
    _path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
        "/carla/" + role_name + "/waypoints",
        10,
        std::bind(&MyPlanningANDControl::path_cb,this,std::placeholders::_1)
    );
    _object_array_subscriber = this->create_subscription<derived_object_msgs::msg::ObjectArray>(
      "/carla/ego_vehicle/objects",
      10,
      std::bind(&MyPlanningANDControl::objects_cb,this,std::placeholders::_1)
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
    _reference_line_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "/carla/" + role_name + "/reference_line",
      10
    );


    _current_ego_state = std::make_shared<VehicleState>();
    _current_ego_state->flag_imu = false;
    _current_ego_state->flag_ode = false;
    _current_ego_state->flag_info = false;

    //绘图
    #ifdef PLOT
    _reference_line_figure_handle = matplot::figure();
    _final_path_figure_handle = matplot::figure();
    #endif
  }

private:
  //参数
  double MIN_DISTANCE_PERCENTAGE ;
  std::string role_name;

  //订阅方以及订阅的数据
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;//里程计订阅方，订阅本车当前位姿与速度
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscriber;//惯性导航订阅方，订阅加速度与角速度
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr _ego_info_subscriber;//定于车辆的车道信息
  std::shared_ptr<VehicleState> _current_ego_state;
  rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr _object_array_subscriber;//对象序列，包含本车和障碍物
  std::vector<derived_object_msgs::msg::Object> _object_arrry;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _target_speed_subscriber;//期望速度订阅方
  double _reference_speed;//期望速度,单位为km/h

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscriber;//路径订阅方
  std::shared_ptr<std::vector<PathPoint>> _global_path;//全局路径存储器

  //发布方
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr _control_cmd_publisher;//控制指令发布方
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _targer_pose_publisher;//目标点发布方
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _reference_line_publisher;


  //控制环节
  std::shared_ptr<LongitudinalPIDController> _longitudinal_pid_controller;//纵向控制器
  std::shared_ptr<LateralPIDController> _lateral_pid_controller;//横向PID控制器
  std::shared_ptr<LaterLQRController> _lateral_lqr_controller;//横向LQR控制器
  rclcpp::TimerBase::SharedPtr _control_timer;//控制器时钟
  double _control_time_step;

  //规划环节
  double _planning_time_step;
  rclcpp::TimerBase::SharedPtr _planning_timer;
  std::shared_ptr<ReferenceLine> _reference_line_generator;//参考线生成器
  std::shared_ptr<std::vector<PathPoint>> _reference_line;
  std::shared_ptr<EMPlanner> _emplanner;
  std::vector<TrajectoryPoint> _trajectory;

  //绘图
  #ifdef PLOT
  size_t _count_plot = 0;
  matplot::figure_handle _reference_line_figure_handle;
  matplot::figure_handle _final_path_figure_handle;
  #endif


public:
  //*******************************************************
  //回调函数
  //里程计订阅方回调函数，获取当前本车位姿与速度
  void odometry_cb(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    _current_ego_state->x = msg->pose.pose.position.x;
    _current_ego_state->y = msg->pose.pose.position.y;
    _current_ego_state->z = msg->pose.pose.position.z;
    _current_ego_state->v = std::sqrt(std::pow(msg->twist.twist.linear.x,2) 
                            +  std::pow(msg->twist.twist.linear.y,2)
                            +  std::pow(msg->twist.twist.linear.z,2));
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg->pose.pose.orientation,tf_q);
    double roll,pitch,yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll,pitch,yaw);
    _current_ego_state->heading = tf2NormalizeAngle(yaw);
    _current_ego_state->flag_ode = true;
    //里程计的单位是m/s，转化为km/h
  }

  void imu_cb(sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    if (imu_msg->linear_acceleration.x >= 10 || imu_msg->linear_acceleration.y >= 10)
    {
      return;
    }
    
    _current_ego_state->ax = imu_msg->linear_acceleration.x;
    _current_ego_state->ay = imu_msg->linear_acceleration.y;
    _current_ego_state->flag_imu = true;
  }

  void ego_info_cb(carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr ego_info)
  {
    _current_ego_state->id = ego_info->id;
    _current_ego_state->flag_info = true;
  }

  //速度指令订阅方回调函数，获取有ad_agent发布的速度指令
  void target_speed_cb(std_msgs::msg::Float64::SharedPtr msg)
  {
    _reference_speed = msg->data;
  }

  void path_cb(nav_msgs::msg::Path::SharedPtr waypoints)
  {
    RCLCPP_INFO(this->get_logger(),"接收到全局路径信息......");
    for (auto &&pose : waypoints->poses)
    {

      PathPoint temp_path_point;
      temp_path_point.x = pose.pose.position.x;
      temp_path_point.y = pose.pose.position.y;
      _global_path->push_back(temp_path_point);

    }
    Calculate_heading_and_kappa(_global_path); 
  }//全局路径订阅回调


  void objects_cb(derived_object_msgs::msg::ObjectArray::SharedPtr object_array)
  {
    _object_arrry.clear();
    for (auto &&object : object_array->objects)
    {
      _object_arrry.push_back(object);
    }
    
  }



  void planning_run_step()//单步规划
  {
    if(_global_path->empty())//等待接受到路径
    {
      RCLCPP_INFO(this->get_logger(),"等待全局路径信息......");
      return;
    }
    //等待接受到主车信息
    if(_current_ego_state->flag_imu == false || _current_ego_state->flag_ode == false || _current_ego_state->flag_info == false)
    {
      RCLCPP_INFO(this->get_logger(),"等待主车信息......");
      return;
    }

    //获取参考线
    _reference_line->clear();
    if(_reference_line_generator->run_step(_current_ego_state,_global_path,_reference_line))
    {
      #ifdef MAIN_DEBUG
      RCLCPP_INFO(this->get_logger(),"参考线生成成功！");
      #endif
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),"参考线生成失败！");
    }

    //将参考线发布出去
    nav_msgs::msg::Path nav_reference_line;
    for (auto &&path_point : *_reference_line)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = path_point.x;
      pose_stamped.pose.position.y = path_point.y;
      pose_stamped.pose.position.z = 0.0;
      nav_reference_line.poses.push_back(pose_stamped);
      //RCLCPP_INFO(this->get_logger(),"参考线(%.2f,%.2f)",path_point.x,path_point.y);
    }
    
    _reference_line_publisher->publish(nav_reference_line);
    RCLCPP_INFO(this->get_logger(),"mian错误定位1");
    //调用emplanner获取轨迹
    auto current_time = this->get_clock()->now();
    _emplanner->planning_run_step(_reference_line,_current_ego_state,_object_arrry,_trajectory);
    
    RCLCPP_INFO(this->get_logger(),"mian错误定位2");
    #ifdef PLOT
    //绘图，因为rviz的可视化没整明白
    if(_count_plot % 20 == 0)
    {
      matplot::figure(_reference_line_figure_handle);
      matplot::cla();
      std::vector<double> global_x,global_y,reference_x,reference_y,trajectory_x,trajectory_y;
      for (auto &&path_point : *_global_path)
      {
        global_x.push_back(path_point.x);
        global_y.push_back(path_point.y);
      }

      //matplot::hold(false);
      matplot::plot(global_x,global_y,"go");
      matplot::hold(true);
      for (auto &&path_point : *_reference_line)
      {
        reference_x.push_back(path_point.x);
        reference_y.push_back(path_point.y);
      }
      matplot::plot(reference_x,reference_y,"--rx");

      int plot_index = 0;
      for (auto &&trajectory_point : _trajectory)
      {
        if (plot_index % 10 == 0)
        {
          trajectory_x.emplace_back(trajectory_point.x);
          trajectory_y.emplace_back(trajectory_point.y);
        }
        plot_index ++ ;

      }
      
      matplot::plot(trajectory_x, trajectory_y,"-b*");

      matplot::plot({_current_ego_state->x}, {_current_ego_state->y}, "ro");
      matplot::hold(false);
      //h->draw();
    }
    _count_plot++ ;

    #endif
    RCLCPP_INFO(this->get_logger(),"main错误定位3");
  }

  void control_run_step()//单步控制
  {
    //没收到路径信息
    if(_current_ego_state->flag_imu == false || _current_ego_state->flag_ode == false ||_current_ego_state->flag_info == false)
    {
      RCLCPP_INFO(this->get_logger(),"等待主车信息......");
      return;
    }
    if(_reference_line->empty())
    {
      RCLCPP_INFO(this->get_logger(),"等待参考线信息......");
      return;
    }
    if (_trajectory.empty())
    {
      RCLCPP_INFO(this->get_logger(),"等待轨迹信息......");
      return;
    }
    
    //目标速度为0
    if(_reference_speed == 0.0)
    {
      emergency_stop();
      return;
    }

    //转换为轨迹点
    //确定匹配点
    // TrajectoryPoint match_point;
    // double min_distance_match_point = 1e10;
    // double distance_match_point = 0;
    // int match_point_index = 0;
    // for (int i = 0; i < (int)_reference_line->size(); i++)
    // {
    //   distance_match_point = std::hypot(_current_ego_state->x - _reference_line->at(i).x,
    //                                     _current_ego_state->y - _reference_line->at(i).y);
    //   if(distance_match_point < min_distance_match_point)
    //   {
    //     min_distance_match_point = distance_match_point;
    //     match_point_index = i;
    //   } 
    // }
    // match_point.x = _reference_line->at(match_point_index).x;
    // match_point.y = _reference_line->at(match_point_index).y;
    // match_point.heading = _reference_line->at(match_point_index).heading;
    // match_point.kappa = _reference_line->at(match_point_index).kappa;
    // match_point.v = _reference_speed;

    //由轨迹搜索出目标点
    TrajectoryPoint target_point;
    double cur_time = this->now().seconds();
    double predicted_time = cur_time + 0.2;
    int target_point_index = -1;
    // RCLCPP_INFO(this->get_logger(),"当前时刻:%.3f",cur_time.seconds());
    // int index = 1;
    // for (size_t i = 0; i < _trajectory->size(); i++)
    // {
    //   RCLCPP_INFO(this->get_logger(), "(序号%d:,x:%.3f, y:%.3f, heading:%.3f, v:%.3f, a_tau:%.3f, time_stampe:%.3f)",
    //   index,_trajectory->at(i).x,_trajectory->at(i).y,_trajectory->at(i).heading,_trajectory->at(i).v,_trajectory->at(i).a_tau,_trajectory->at(i).time_stamped.seconds());
    //   index++;
    // }
    
    for (int i = 0; i < (int)_trajectory.size() - 1; i++)
    {
      if (predicted_time >= _trajectory.at(i).time_stamped && predicted_time < _trajectory.at(i+1).time_stamped)
      {
        target_point_index = i;
        break;
      }
    }
    double delta_t = (_trajectory.at(target_point_index+1).time_stamped - _trajectory.at(target_point_index).time_stamped);
    double dt = predicted_time - _trajectory.at(target_point_index).time_stamped;

    double k_x = (_trajectory.at(target_point_index+1).x - _trajectory.at(target_point_index).x)/delta_t;
    target_point.x = _trajectory.at(target_point_index).x + k_x*dt;

    double k_y = (_trajectory.at(target_point_index+1).y - _trajectory.at(target_point_index).y)/delta_t;
    target_point.y = _trajectory.at(target_point_index).y + k_y*dt;

    double k_v = (_trajectory.at(target_point_index+1).v - _trajectory.at(target_point_index).v)/delta_t;
    target_point.v = _trajectory.at(target_point_index).v + k_v*dt;

    double k_heading = (_trajectory.at(target_point_index+1).heading - _trajectory.at(target_point_index).heading)/delta_t;
    target_point.heading = _trajectory.at(target_point_index).heading + k_heading*dt;

    double k_a_tau = (_trajectory.at(target_point_index+1).a_tau - _trajectory.at(target_point_index).a_tau)/delta_t;    
    target_point.a_tau = _trajectory.at(target_point_index).a_tau + k_a_tau*dt;

    #ifdef MAIN_DEBUG
    RCLCPP_INFO(this->get_logger(),"当前位姿(%.3f,%.3f,%.3f,%.3f),期望位姿(%.3f,%.3f,%.3f,%.3f)",
                _current_ego_state->x,_current_ego_state->y,_current_ego_state->heading,_current_ego_state->v,
                target_point.x,target_point.y,target_point.heading, target_point.v);
    // RCLCPP_INFO(this->get_logger(),"参考线起点位姿(%.3f,%.3f,%.3f),参考线终点位姿(%.3f,%.3f,%.3f)",
    //             _reference_line->at(0).x,_reference_line->at(0).y,_reference_line->at(0).heading,
    //             _reference_line->back().x,_reference_line->back().y,_reference_line->back().heading);

    #endif
    
    //RCLCPP_INFO(this->get_logger(),"目标位置(%.2f,%.2f)",target_pose.pose.position.x,target_pose.pose.position.y);
    //****发布标记
    visualization_msgs::msg::Marker marker;
    marker.type = 0;
    marker.header.frame_id = "map";
    marker.scale.x = 1.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 255;
    marker.color.a = 1.0;
    _targer_pose_publisher->publish(marker);
    

    //计算并发布控制指令
    carla_msgs::msg::CarlaEgoVehicleControl control_msg;
    control_msg.throttle = _longitudinal_pid_controller->run_step(target_point.v,_current_ego_state->v);
    //control_msg.steer = _lateral_pid_controller->run_step(target_pose,*_current_ego_state);
    control_msg.steer = _lateral_lqr_controller->run_step(target_point,*_current_ego_state,_control_time_step);
    control_msg.hand_brake = false;
    control_msg.manual_gear_shift = false;
    control_msg.reverse = false;
    
    _control_cmd_publisher->publish(control_msg);


  }

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

  rclcpp::spin(std::make_shared<MyPlanningANDControl>());

  rclcpp::shutdown();
  
  return 0;
}
