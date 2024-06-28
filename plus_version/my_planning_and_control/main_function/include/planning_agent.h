#pragma once

#include <deque>
#include <vector>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "matplot/matplot.h"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "std_msgs/msg/float64.hpp"

#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_info.hpp"
#include "carla_waypoint_types/srv/get_waypoint.hpp"
#include "pnc_msgs/msg/trajectory.hpp"
#include "pnc_msgs/msg/trajectory_point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_eigen/tf2_eigen.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "derived_object_msgs/msg/object.hpp"
#include "derived_object_msgs/msg/object_array.hpp"

#include "reference_line.h"
#include "emplanner.h"
