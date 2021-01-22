cmake_minimum_required(VERSION 3.5)
project(spot_micro_motion_cmd)

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
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # message_generation in ROS1
find_package(rosidl_default_runtime REQUIRED)     # message_runtime in ROS1
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(i2cpwmboard REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM
    ${EIGEN3_INCLUDE_DIRS} 
    ${geometry_msgs_INCLUDE_DIRS} 
    ${std_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_msgs_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${i2cpwmboard_INCLUDE_DIRS}
    )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include/spot_micro_motion_cmd
  lib/spot_micro_kinematics_cpp/include
  src/smfsm
  src/rate_limited_first_order_filter
)

link_directories(${CMAKE_SOURCE_DIR}/lib)

add_library(smfsm 
            src/smfsm/spot_micro_state.cpp 
            src/smfsm/spot_micro_idle.cpp
            src/smfsm/spot_micro_stand.cpp
			src/smfsm/spot_micro_transition_stand.cpp
			src/smfsm/spot_micro_transition_idle.cpp
			src/smfsm/spot_micro_walk.cpp
)

target_link_libraries(smfsm
    spot_micro_kinematics
    ${geometry_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${tf2_LIBRARIES}
    ${tf2_msgs_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/spot_micro_motion_cmd.cpp src/spot_micro_motion_cmd_node.cpp src/utils.cpp)

target_link_libraries(${PROJECT_NAME}_node
smfsm
spot_micro_kinematics
${geometry_msgs_LIBRARIES}
${rclcpp_LIBRARIES}
${tf2_LIBRARIES}
${tf2_msgs_LIBRARIES}
)


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
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_msgs)
ament_export_dependencies(tf2)
ament_package()
