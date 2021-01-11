cmake_minimum_required(VERSION 3.5)
project(spotmicro)

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
find_package(geometry_msgs REQUIRED)
find_package(spotmicro REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)  # message_generation in ROS1
find_package(rosidl_default_runtime REQUIRED)              # message_runtime in ROS1
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
rosidl_generate_interfaces(${PROJECT_NAME}
  "src/spotmicro/msg/Num.msg"
  "src/spotmicro/msg/JointAngles.msg"
  "src/spotmicro/msg/JointPulse.msg"
  "src/spotmicro/msg/AgentData.msg"
  "src/spotmicro/msg/IMUdata.msg"
  "src/spotmicro/msg/MiniCmd.msg"
  "src/spotmicro/msg/JoyButtons.msg"
  "src/spotmicro/msg/ContactData.msg"
  "src/spotmicro/srv/AddThreeInts.srv"
  "src/spotmicro/srv/CalibServo.srv"
 )
 add_executable(talker src/spotmicro/src/publish_member_function.cpp)
 ament_target_dependencies(talker rclcpp spotmicro)         # CHANGE
 
 add_executable(listener src/spotmicro/src/subscriber_member_function.cpp)
 ament_target_dependencies(listener rclcpp spotmicro)  
 
 install(TARGETS
   talker
   listener
   DESTINATION lib/${PROJECT_NAME})

   add_executable(server src/spotmicro/src/add_three_ints_server.cpp)
   ament_target_dependencies(server
     rclcpp spotmicro)                      #CHANGE
   
   add_executable(client src/spotmicro/src/add_three_ints_client.cpp)
   ament_target_dependencies(client
   rclcpp spotmicro)    
   
   install(TARGETS
   server
   client
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
