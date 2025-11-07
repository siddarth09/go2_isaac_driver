cmake_minimum_required(VERSION 3.8)
project(go2_isaac_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
  "srv/ComputeIK.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs visualization_msgs 
)
include_directories(include)


# INVERSEKINEMATICS 
add_executable(go2_ik_service src/inversekinematics.cpp)
ament_target_dependencies(go2_ik_service rclcpp sensor_msgs)
rosidl_target_interfaces(go2_ik_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Stand Control
add_executable(go2_stand src/StandControl.cpp)
ament_target_dependencies(go2_stand rclcpp sensor_msgs tf2)
rosidl_target_interfaces(go2_stand
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Sit Control
add_executable(go2_sit src/SitControl.cpp)
ament_target_dependencies(go2_sit rclcpp sensor_msgs tf2)
rosidl_target_interfaces(go2_sit
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS 
  go2_ik_service
  go2_stand
  go2_sit

  DESTINATION lib/${PROJECT_NAME})



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
