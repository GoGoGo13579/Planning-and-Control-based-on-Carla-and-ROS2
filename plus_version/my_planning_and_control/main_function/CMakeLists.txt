cmake_minimum_required(VERSION 3.5)
project(my_pnc)
set(CMAKE_CXX_STANDARD 11)

add_compile_definitions(MAIN_DEBUG)
add_compile_definitions(PLOT)
add_compile_definitions(CONTROL_DEBUG)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_BUILD_TYPE Debug)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(OsqpEigen_DIR /opt/osqp_eigen/lib/cmake/OsqpEigen)
set(osqp_DIR /opt/osqp/lib/cmake/osqp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(derived_object_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pnc_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(carla_waypoint_types REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../controller_modules/src SRC_CONTROL)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../planner_modules/src SRC_PLANNING)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/../general_modules/src SRC_GENERAL)
#message(ERROR ${SRC_PLANNING})
#配置规划代理的可执行文件
add_executable(planning_agent src/planning_agent.cpp ${SRC_CONTROL} ${SRC_PLANNING} ${SRC_GENERAL})

target_include_directories(planning_agent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../controller_modules/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../general_modules/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../planner_modules/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  planning_agent
  "rclcpp"
  "carla_msgs"
  "derived_object_msgs"
  "nav_msgs"
  "std_msgs"
  "visualization_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "pnc_msgs"
  "carla_waypoint_types"
)

target_link_libraries(
  planning_agent
  OsqpEigen::OsqpEigen
  Matplot++::matplot
)

install(TARGETS planning_agent
  DESTINATION lib/${PROJECT_NAME})

#配置控制代理的可执行文件
add_executable(control_agent src/control_agent.cpp ${SRC_CONTROL} ${SRC_GENERAL})

target_include_directories(control_agent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../controller_modules/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../general_modules/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies( #这是把规划直接粘贴过来了，其实用不上这么多库
  control_agent
  "rclcpp"
  "carla_msgs"
  "derived_object_msgs"
  "nav_msgs"
  "std_msgs"
  "visualization_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "pnc_msgs"
  "carla_waypoint_types"
)

target_link_libraries(
  control_agent
  OsqpEigen::OsqpEigen
  Matplot++::matplot
)

install(TARGETS control_agent
  DESTINATION lib/${PROJECT_NAME})

#安装launch文件
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
