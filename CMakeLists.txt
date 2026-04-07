cmake_minimum_required(VERSION 3.8)
project(human_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)

#v1
add_executable(leg_finder_node src/leg_finder_node.cpp)
add_executable(human_follower_nodeV2 src/human_follower_nodeV2.cpp)

#v2
add_executable(leg_finder_nodev2 src/leg_finder_node.cpp)
add_executable(human_follower_node src/human_follower_node.cpp)
add_executable(human_tracking_node src/human_tracking_node.cpp)

ament_target_dependencies(leg_finder_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  Eigen3
)

ament_target_dependencies(leg_finder_nodev2
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  Eigen3
)

ament_target_dependencies(human_follower_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  Eigen3
)

ament_target_dependencies(human_follower_nodeV2
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  Eigen3
)

ament_target_dependencies(human_tracking_node
  rclcpp
  std_msgs
  std_srvs
)

install(TARGETS 
  leg_finder_node
  leg_finder_nodev2
  human_follower_node
  human_follower_nodeV2
  human_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
