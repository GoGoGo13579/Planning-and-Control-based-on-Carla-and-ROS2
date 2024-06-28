#include "common.h"


// TrajectoryPoint& TrajectoryPoint::operator=(const TrajectoryPoint& other)
// {
//     this->x = other.x;
//     this->y = other.y;
//     this->heading = other.heading;
//     this->kappa = other.kappa;
//     this->v = other.v;
//     this->ax = other.ax;
//     this->ay = other.ay;
//     this->a_tau = other.a_tau;
//     this->time_stamped = other.time_stamped;

//     return *this;
// }

// TrajectoryPoint::TrajectoryPoint(const TrajectoryPoint& other)
// {
//     this->x = other.x;
//     this->y = other.y;
//     this->heading = other.heading;
//     this->kappa = other.kappa;
//     this->v = other.v;
//     this->ax = other.ax;
//     this->ay = other.ay;
//     this->a_tau = other.a_tau;
//     this->time_stamped = other.time_stamped;
// }
void Calculate_heading_and_kappa(std::vector<PathPoint>& path)
{
    int n = path.size();
    std::vector<double> dx;
    std::vector<double> dy;
    std::vector<double> ds;
    dx.resize(n);
    dy.resize(n);
    ds.resize(n);
    for (int i = 0; i < (int)path.size(); i++)
    {
        if (i == 0)
        {
            dx[i] = path.at(i+1).x - path.at(i).x;
            dy[i] = path.at(i+1).y - path.at(i).y;
        }
        else if (i == (int)path.size()-1)
        {
            dx[i] = path.at(i).x - path.at(i-1).x;
            dy[i] = path.at(i).y - path.at(i-1).y;
        }
        else
        {
            dx[i] = (path.at(i+1).x - path.at(i-1).x)/2.0;
            dy[i] = (path.at(i+1).y - path.at(i-1).y)/2.0;
        }
        
        ds[i] = std::hypot(dx[i],dy[i]);
        path.at(i).heading = std::atan2(dy[i],dx[i]);
        
    }

    std::vector<double> dtheta;
    dtheta.resize(n);
    
    for (int i = 0; i < (int)path.size(); i++)
    {
        if (i == 0)
        {
            dtheta[i] = path.at(i+1).heading - path.at(i).heading;
        }
        else if (i == (int)path.size()-1)
        {
            dtheta[i] = path.at(i).heading - path.at(i-1).heading;
        }
        else
        {
            dtheta[i] = (path.at(i+1).heading - path.at(i-1).heading)/2.0;
        }
        
        //这里可能存在多值问题
        path.at(i).kappa = dtheta[i]/ds[i];
    }
    
}

void calculate_projection_point(const std::vector<PathPoint>& path, const Eigen::Vector2d point, int& match_point_index, PathPoint& projection_point)
{
    /*
    计算给定点在路径上的投影点
    1.输入参数
    */
    //1.搜索投影点
    double min_distance = std::numeric_limits<double>::max();
    size_t count_num = 0;
    for (size_t i = 0; i < path.size(); i++)
    {
        double distance = std::hypot(path.at(i).x - point[0], path.at(i).y - point[1]);
        if (distance < min_distance)
        {
            match_point_index = i;
            min_distance = distance;
            count_num = 0;
        }
        count_num ++;
        if (count_num > 30)
        {
            break;
        }
    }
    //2.由匹配点计算投影点
    Eigen::Vector2d tau_m; //匹配点切线向量
    Eigen::Vector2d d;//匹配点指向主车的向量 
    Eigen::Vector2d match_point;//匹配点位置向量
    tau_m << std::cos(path.at(match_point_index).heading),std::sin(path.at(match_point_index).heading);
    match_point << path.at(match_point_index).x, path.at(match_point_index).y;
    d = point - match_point;
    //2.1计算x，y
    auto v_pro = match_point + (d.dot(tau_m))*tau_m;
    projection_point.x = v_pro[0];
    projection_point.y = v_pro[1];
    //2.2计算heading和kappa
    projection_point.heading = path.at(match_point_index).heading + path.at(match_point_index).kappa *(d.dot(tau_m));
    projection_point.kappa = path.at(match_point_index).kappa;
 
}
//计算投影点的重载版本
void calculate_projection_point(const std::vector<PathPoint>& path, const TrajectoryPoint& trajectory_point, int& match_point_index,
                                 PathPoint& match_point, PathPoint& projection_point)
{
    double min_distance = std::numeric_limits<double>::max();
    size_t count_num = 0;
    for (size_t i = 0; i < path.size(); i++)
    {
        double distance = std::hypot(path.at(i).x - trajectory_point.x, path.at(i).y - trajectory_point.y);
        if (distance < min_distance)
        {
            match_point_index = i;
            min_distance = distance;
            count_num = 0;
        }
        count_num ++;
        if (count_num > 30)
        {
            break;
        }
    }
    //2.由匹配点计算投影点
    Eigen::Vector2d tau_m; //匹配点切线向量
    Eigen::Vector2d d;//匹配点指向主车的向量 
    Eigen::Vector2d v_match_point;//匹配点位置向量
    tau_m << std::cos(path.at(match_point_index).heading),std::sin(path.at(match_point_index).heading);
    v_match_point << path.at(match_point_index).x, path.at(match_point_index).y;
    d << trajectory_point.x - v_match_point[0], trajectory_point.y - v_match_point[1];
    //2.1计算x，y，赋值
    auto v_pro = v_match_point + (d.dot(tau_m))*tau_m;
    projection_point.x = v_pro[0];
    projection_point.y = v_pro[1];
    //2.2计算heading和kappa，赋值
    projection_point.heading = path.at(match_point_index).heading + path.at(match_point_index).kappa *(d.dot(tau_m));
    projection_point.kappa = path.at(match_point_index).kappa;

    //匹配点赋值
    match_point.x = v_match_point[0];
    match_point.y = v_match_point[1];
    match_point.heading = path.at(match_point_index).heading;
    match_point.kappa = path.at(match_point_index).kappa;
 
}

std::vector<double> calculate_index_to_s(const std::vector<PathPoint>& path, std::shared_ptr<VehicleState> vehicle_state)
{
    /*
    将路径转换为index2s表，加速后续程序计算
    */
    //1.计算投影点索引
    Eigen::Vector2d host_point(vehicle_state->x, vehicle_state->y);
    int match_point_index;
    PathPoint projection_point;
    calculate_projection_point(path,host_point,match_point_index,projection_point);

    //2.计算初始表，原点为第一个点
    std::vector<double> index2s;
    double distance = 0.0;
    index2s.push_back(distance);
    for (size_t i = 1; i < path.size(); i++)
    {
        distance += std::hypot(path.at(i).x - path.at(i-1).x, path.at(i).y - path.at(i-1).y);
        index2s.push_back(distance);
    }

    //3.将原点改至投影点
    //判断投影点在匹配点的前方还是后方,获取投影点所对应的s
    Eigen::Vector2d match_to_pro(projection_point.x - path.at(match_point_index).x,
                                projection_point.y - path.at(match_point_index).y);
    Eigen::Vector2d tau_match(std::cos(path.at(match_point_index).heading), std::sin(path.at(match_point_index).heading));
    double delta_s = 0;
    if(match_to_pro.dot(tau_match) > 0.0)//投影点在匹配点前方
    {
        delta_s = index2s[match_point_index] + match_to_pro.norm();
    }
    else if(match_to_pro.dot(tau_match) < 0.0)//投影点在匹配点后方
    {
        delta_s = index2s[match_point_index] - match_to_pro.norm();
    }
    else
    {
        delta_s = index2s[match_point_index];
    }

    //将原有的index2s表平移
    for (auto && element : index2s)
    {
        element -= delta_s;
    }

    //4.输出
    return index2s;

}


std::vector<TrajectoryPoint> frenet_to_cartesion(const std::vector<FrenetPoint>& frenet_point_set, const std::vector<PathPoint>& cartesian_path,
                                                 const std::vector<double> cartesian_path_index2s)
{
    std::vector<TrajectoryPoint> trajectory_point_set;

    for (int i = 0; i < (int)frenet_point_set.size(); i++)
    {   
        FrenetPoint frenet_point_host(frenet_point_set[i]);
        //1.寻找匹配点
        int match_point_index = -1;
        if (frenet_point_host.s < cartesian_path_index2s.front())//边界情况
        {
            match_point_index = 0;
        }
        else if(frenet_point_host.s >= cartesian_path_index2s.back())
        {
            match_point_index = cartesian_path_index2s.size()-1;
        }
        else
        {   //循环index2s，找到在s坐标上，离待转化点最近的点
            for (int j = 0; j < (int)cartesian_path_index2s.size() - 1 ; j++)
            {
                if (frenet_point_host.s >= cartesian_path_index2s[j] && frenet_point_host.s < cartesian_path_index2s[j+1])
                {
                    if (std::abs(frenet_point_host.s - cartesian_path_index2s[j]) < std::abs(frenet_point_host.s - cartesian_path_index2s[j+1]) )
                    {
                        match_point_index = j+1;
                        break;
                    }
                    else
                    {
                        match_point_index = j;
                        break;
                    }   
                }   
            }
        }
        //匹配点，位置向量，航向角，切向量，曲率
        Eigen::Vector2d p_match_point(cartesian_path.at(match_point_index).x, cartesian_path.at(match_point_index).y);
        double heading_match_point = cartesian_path.at(match_point_index).heading;
        Eigen::Vector2d tau_match_point(std::cos(heading_match_point), std::sin(heading_match_point));
        double kappa_match_point = cartesian_path.at(match_point_index).kappa;
        
        //计算投影点
        Eigen::VectorXd p_projection_point = p_match_point + (frenet_point_host.s - cartesian_path_index2s[match_point_index])*tau_match_point;
        double kappa_projection_point = kappa_match_point;
        double heading_projection_point = heading_match_point + (frenet_point_host.s - cartesian_path_index2s[match_point_index])*kappa_projection_point;
        Eigen::Vector2d nor_projection_point(-std::sin(heading_projection_point), std::cos(heading_projection_point));

        //坐标转换
        TrajectoryPoint trajectory_point_host;
        Eigen::VectorXd p_host = p_projection_point + frenet_point_host.l*nor_projection_point;
        trajectory_point_host.x = p_host[0];
        trajectory_point_host.y = p_host[1];
        double c = 1.0 - kappa_projection_point*frenet_point_host.l;
        trajectory_point_host.heading = std::atan2(frenet_point_host.l_prime, c) + heading_projection_point;
        double delta_heading = trajectory_point_host.heading - heading_projection_point;
        trajectory_point_host.kappa = ((frenet_point_host.l_prime_prime + kappa_projection_point*frenet_point_host.l_prime*std::tan(delta_heading))*std::pow(std::cos(delta_heading),2)/c + kappa_projection_point)*std::cos(delta_heading)/c;

        trajectory_point_set.emplace_back(trajectory_point_host);
    }   

    return trajectory_point_set;

}

void cartesion_set_to_frenet_set(const std::vector<derived_object_msgs::msg::Object>& object_set, const std::vector<TrajectoryPoint>& trajectory, const TrajectoryPoint& original_point, std::vector<FrenetPoint>& frenet_set)
{
    //1.若为空，直接退出
    if (object_set.empty())
    {
        return;
    }

    //2.计算index2s表
    //2.1原始index2s表
    std::vector<double> index2s;
    index2s.emplace_back(0.0);
    for (size_t i = 1; i < trajectory.size(); i++)
    {
        index2s.emplace_back((std::hypot(trajectory[i].x - trajectory[i-1].x, trajectory[i].y - trajectory[i-1].y)) + index2s.back());
    }
    //2.2搜索匹配点
    int match_point_index = -1;
    double min_distace = std::numeric_limits<double>::max();
    for(int j = 0 ; j < (int)trajectory.size() ; j++)
    {
        double current_distace = hypot(trajectory[j].x - original_point.x, trajectory[j].y - original_point.y);
        if ( current_distace < min_distace )
        {
            min_distace = current_distace;
            match_point_index = j;
        }
    }
    //2.3由匹配点计算投影点
    //匹配点信息：位置向量，航向角，曲率，切向量，法向量
    Eigen::Vector2d match_point_p(trajectory[match_point_index].x, trajectory[match_point_index].y);
    double match_point_heading = trajectory[match_point_index].heading;
    double match_point_kappa = trajectory[match_point_index].kappa;
    Eigen::Vector2d match_point_tau(std::cos(match_point_heading), std::sin(match_point_heading));
    Eigen::Vector2d match_point_nor(-std::sin(match_point_heading), std::cos(match_point_heading));

    Eigen::Vector2d match_point_to_original_point(original_point.x - match_point_p[0], original_point.y - match_point_p[1]);
    //计算投影点信息：位置向量，航向角，曲率，切向量，法向量
    Eigen::Vector2d projection_point_p = match_point_p + match_point_tau*(match_point_tau.dot(match_point_to_original_point));
    double projection_point_heading = match_point_heading + match_point_kappa*(match_point_tau.dot(match_point_to_original_point));
    double projection_point_kappa = match_point_kappa;
    Eigen::Vector2d projection_point_tau(std::cos(projection_point_heading), std::sin(projection_point_heading));
    Eigen::Vector2d projection_point_nor(-std::sin(projection_point_heading), std::cos(projection_point_heading));

    //2.4修正index2s表
    Eigen::Vector2d match_point_to_projection_point(projection_point_p[0] - match_point_p[0],
                                                    projection_point_p[1] - match_point_p[1]);//匹配点指向投影点的向量
    double delta_s = index2s[match_point_index];//修正量
    if (match_point_to_projection_point.dot(match_point_tau) > 0)//判断投影点在匹配点的前面还是后面
    {
        delta_s += match_point_to_projection_point.norm();
    }
    else
    {
        delta_s -= match_point_to_projection_point.norm();
    }

    for (size_t i = 0; i < index2s.size(); i++)//遍历每一个元素，逐个修正
    {
        index2s[i] -= delta_s;
    }

    //逐个坐标转化
    for (auto && object : object_set)
    {

        TrajectoryPoint host = object_to_trajectory_point(object);
        
        //带转化对象相关信息：位置向量，航向角，曲率，切向量，方向向量
        Eigen::Vector2d host_p(host.x, host.y);
        double host_heading = host.heading;
        double host_v = host.v;
        Eigen::Vector2d host_a(host.ax, host.ay);
        Eigen::Vector2d host_tau(std::cos(host_heading), std::sin(host_heading));
        Eigen::Vector2d host_nor(-std::sin(host_heading), std::cos(host_heading));

        //搜索投影点
        match_point_index = -1;
        min_distace = std::numeric_limits<double>::max();
        for(int j = 0 ; j < (int)trajectory.size() ; j++)
        {
            double current_distace = hypot(trajectory[j].x - host.x, trajectory[j].y - host.y);
            if ( current_distace < min_distace )
            {
                min_distace = current_distace;
                match_point_index = j;
            }
        }

        //匹配点信息：位置向量，航向角，曲率，切向量，法向量
        match_point_p << trajectory[match_point_index].x, trajectory[match_point_index].y;
        match_point_heading = trajectory[match_point_index].heading;
        match_point_kappa = trajectory[match_point_index].kappa;
        match_point_tau << std::cos(match_point_heading), std::sin(match_point_heading);
        match_point_nor << -std::sin(match_point_heading), std::cos(match_point_heading);
        //匹配点到待转化点向量
        Eigen::Vector2d match_point_to_host(host_p[0] - match_point_p[0], host_p[1] - match_point_p[1]);
        //投影点信息：位置向量，航向角，曲率，切向量，法向量
        projection_point_p = match_point_p + match_point_tau*(match_point_tau.dot(match_point_to_host));
        projection_point_heading = match_point_heading + match_point_kappa*(match_point_tau.dot(match_point_to_host));
        projection_point_kappa = match_point_kappa;
        projection_point_tau << std::cos(projection_point_heading), std::sin(projection_point_heading);
        projection_point_nor << -std::sin(projection_point_heading), std::cos(projection_point_heading);
        //坐标转化
        double l = (host_p - projection_point_p).dot(projection_point_nor);
        double c = 1 - projection_point_kappa*l;
        double s_dot = host_v * (host_tau.dot(projection_point_tau))/c;
        double l_dot = host_v * (host_tau.dot(projection_point_nor));
        double l_prime = l_dot / (s_dot + 1e-10);
        double s_dot_dot = host_a.dot(projection_point_tau)/c + projection_point_kappa*s_dot*s_dot*l_prime/c + s_dot*s_dot*projection_point_kappa*l_prime/c;
        double l_dot_dot = host_a.dot(projection_point_nor) - projection_point_kappa*c*s_dot*s_dot;
        double l_prime_prime = (l_dot_dot - l_prime*s_dot_dot)/(s_dot*s_dot + 1e-3);
        double s = index2s[match_point_index];

        match_point_to_projection_point << projection_point_p[0] - match_point_p[0],projection_point_p[1] - match_point_p[1];//匹配点指向投影点的向量
        if (match_point_to_projection_point.dot(match_point_tau) > 0)//判断投影点在匹配点的前面还是后面
        {
            s+= match_point_to_projection_point.norm();
        }
        else
        {
            s -= match_point_to_projection_point.norm();
        }

        FrenetPoint host_frenet;
        host_frenet.s = s;
        host_frenet.l = l;
        host_frenet.s_dot = s_dot;
        host_frenet.l_dot = l_dot;
        host_frenet.l_prime = l_prime;
        host_frenet.s_dot_dot = s_dot_dot;
        host_frenet.l_dot_dot = l_dot_dot;
        host_frenet.l_prime_prime = l_prime_prime;

        frenet_set.emplace_back(host_frenet);
    }
}

void cartesion_set_to_frenet_set(const std::vector<derived_object_msgs::msg::Object>& object_set, const std::vector<PathPoint>& path, std::shared_ptr<VehicleState> vehicle_state, std::vector<FrenetPoint>& frenet_set)
{
    //调用另一个函数，把相应数据转化一下
    std::vector<TrajectoryPoint> trajectory;
    for (auto &&path_point : path)
    {
        TrajectoryPoint trajectory_point;
        trajectory_point.x = path_point.x;
        trajectory_point.y = path_point.y;
        trajectory_point.heading = path_point.heading;
        trajectory_point.kappa = path_point.kappa;

        trajectory.emplace_back(trajectory_point);
    }

    TrajectoryPoint original_point = vehicle_state_to_trajectory_point(vehicle_state);

    cartesion_set_to_frenet_set(object_set, trajectory, original_point, frenet_set);
    
}

void cartesion_set_to_frenet_set(const TrajectoryPoint& trajectory_point, const std::vector<PathPoint>& path, std::shared_ptr<VehicleState> vehicle_state, std::vector<FrenetPoint>& frenet_set)
{
    derived_object_msgs::msg::Object object;
    object.pose.position.x = trajectory_point.x;
    object.pose.position.y = trajectory_point.y;
    object.pose.position.z = 0.0;
    object.accel.linear.x = trajectory_point.ax;
    object.accel.linear.y = trajectory_point.ay;
    tf2::Quaternion tf_q;
    tf_q.setEuler(0.0, 0.0, trajectory_point.heading);
    object.pose.orientation.x = tf_q.x();
    object.pose.orientation.y = tf_q.y();
    object.pose.orientation.z = tf_q.z();
    object.pose.orientation.w = tf_q.w();

    std::vector<derived_object_msgs::msg::Object> object_set;
    object_set.emplace_back(object);

    cartesion_set_to_frenet_set(object_set, path, vehicle_state, frenet_set);
}

void cartesion_set_to_frenet_set(const std::shared_ptr<VehicleState>& ego_state,
                                 const std::vector<TrajectoryPoint>& trajectory,
                                 std::vector<FrenetPoint>& frenet_set)
{
    derived_object_msgs::msg::Object ego_obj;
    ego_obj.pose.position.x = ego_state->x;
    ego_obj.pose.position.y = ego_state->y;
    ego_obj.pose.position.z = 0.0;
    ego_obj.twist.linear.x  = ego_state->v * std::cos(ego_state->heading);
    ego_obj.twist.linear.y  = ego_state->v * std::sin(ego_state->heading);
    ego_obj.twist.linear.z  = 0.0;
    ego_obj.accel.linear.x = ego_state->ax;
    ego_obj.accel.linear.y = ego_state->ay;
    ego_obj.accel.linear.z = 0.0;
    tf2::Quaternion tf_q;
    tf_q.setEuler(0.0, 0.0, ego_state->heading);
    ego_obj.pose.orientation.x = tf_q.x();
    ego_obj.pose.orientation.y = tf_q.y();
    ego_obj.pose.orientation.z = tf_q.z();
    ego_obj.pose.orientation.w = tf_q.w();

    std::vector<derived_object_msgs::msg::Object> obj_set;
    obj_set.emplace_back(ego_obj);

    cartesion_set_to_frenet_set(obj_set, trajectory, trajectory.front(), frenet_set);
}

//对象转换至轨迹点
TrajectoryPoint object_to_trajectory_point(const derived_object_msgs::msg::Object object)
{
    TrajectoryPoint trajectory_point;
    trajectory_point.x = object.pose.position.x;
    trajectory_point.y = object.pose.position.y;
    trajectory_point.v = std::sqrt(std::pow(object.twist.linear.x, 2.0) +
                                   std::pow(object.twist.linear.y, 2.0) +
                                   std::pow(object.twist.linear.z, 2.0));
    trajectory_point.kappa = 0.0;
    trajectory_point.ax = object.accel.linear.x;
    trajectory_point.ay = object.accel.linear.y;

    tf2::Quaternion tf2_q;
    tf2::fromMsg(object.pose.orientation,tf2_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll,pitch,yaw);
    trajectory_point.heading = yaw;

    return trajectory_point;
}

TrajectoryPoint vehicle_state_to_trajectory_point(const std::shared_ptr<VehicleState> vehicle_state)
{
    TrajectoryPoint trajectory_point;
    trajectory_point.x = vehicle_state->x;
    trajectory_point.y = vehicle_state->y;
    trajectory_point.heading = vehicle_state->heading;
    trajectory_point.v = vehicle_state->v;
    trajectory_point.ax = vehicle_state->ay;
    trajectory_point.ay = vehicle_state->ay;

    return trajectory_point;

}