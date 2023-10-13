cmake_minimum_required(VERSION 3.5)
project(custom_msg_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cslam_common_interfaces REQUIRED)
find_package(cslam_common_interfaces REQUIRED)

add_executable(custom_odom_1 src/custom_odom_1.cpp)
ament_target_dependencies(custom_odom_1 rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_odom_2 src/custom_odom_2.cpp)
ament_target_dependencies(custom_odom_2 rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_odom_3 src/custom_odom_3.cpp)
ament_target_dependencies(custom_odom_3 rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_odom_4 src/custom_odom_4.cpp)
ament_target_dependencies(custom_odom_4 rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_odom_5 src/custom_odom_5.cpp)
ament_target_dependencies(custom_odom_5 rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_odom_6 src/custom_odom_6.cpp)
ament_target_dependencies(custom_odom_6 rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_odom_7 src/custom_odom_7.cpp)
ament_target_dependencies(custom_odom_7 rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_odom_8 src/custom_odom_8.cpp)
ament_target_dependencies(custom_odom_8 rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_loop_closure_no_blockchain src/custom_loop_closure_no_blockchain.cpp)
ament_target_dependencies(custom_loop_closure_no_blockchain rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_loop_closure_free_generation src/custom_loop_closure_free_generation.cpp)
ament_target_dependencies(custom_loop_closure_free_generation rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)
add_executable(custom_loop_closure_forced_generation src/custom_loop_closure_forced_generation.cpp)
ament_target_dependencies(custom_loop_closure_forced_generation rclcpp std_msgs cslam_common_interfaces cslam_common_interfaces)

install(TARGETS
  custom_odom_1
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_odom_2
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_odom_3
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_odom_4
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_odom_5
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_odom_6
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_odom_7
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_odom_8
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_loop_closure_no_blockchain
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_loop_closure_free_generation
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  custom_loop_closure_forced_generation
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
