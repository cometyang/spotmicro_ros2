cmake_minimum_required(VERSION 3.5)
project(i2cpwmboard)

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
find_package(rosidl_default_generators REQUIRED)  # message_generation in ROS1
find_package(rosidl_default_runtime REQUIRED)     # message_runtime in ROS1
find_package(sensor_msgs REQUIRED)
find_package(XMLRPC REQUIRED c++)
find_package(i2cpwmboard REQUIRED)



include_directories(include
   ${geometry_msgs_INCLUDE_DIRS}
   ${XMLRPC_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/JointAngles.msg"
  "msg/JointPulse.msg"
  "msg/AgentData.msg"
  "msg/IMUdata.msg"
  "msg/MiniCmd.msg"
  "msg/JoyButtons.msg"
  "msg/ContactData.msg"
  "msg/Position.msg"
  "msg/PositionArray.msg"
  "msg/Servo.msg"
  "msg/ServoArray.msg"
  "msg/ServoConfig.msg"
  "msg/ServoConfigArray.msg"
  "srv/AddThreeInts.srv"
  "srv/CalibServo.srv"
  "srv/IntValue.srv"
  "srv/DriveMode.srv"
  "srv/ServosConfig.srv"
  "srv/StopServos.srv"

 )




 add_executable(talker src/publisher_member_function.cpp)
 ament_target_dependencies(talker rclcpp i2cpwmboard)         # CHANGE
 
 add_executable(listener src/subscriber_member_function.cpp)
 ament_target_dependencies(listener rclcpp i2cpwmboard)     # CHANGE
 
 add_executable(i2cpwm_board src/i2cpwm_controller.cpp)
 target_link_libraries(i2cpwm_board i2c xmlrpcpp)
 ament_target_dependencies(i2cpwm_board rclcpp std_srvs geometry_msgs i2cpwmboard)     # CHANGE

 install(TARGETS
   talker
   listener
   DESTINATION lib/${PROJECT_NAME})

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
