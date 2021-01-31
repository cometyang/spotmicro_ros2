// Node file to create object and initialising the ROS node
#include "spot_micro_motion_cmd.h" 
#include <iostream>


int main(int argc, char** argv) {
  /* initialising the ROS node creating node handle
  for regestring it to the master and then private node handle to
  handle the parameters */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SpotMicroMotionCmd>();

  
  RCLCPP_INFO(node->get_logger(), "Publish static transforms");
  // Publish static transforms
  node->publishStaticTransforms();
  RCLCPP_INFO(node->get_logger(), "Finish node initialization");
    float duration = node->getNodeConfig().dt;
    float rate = 1.0/duration;
   
    RCLCPP_INFO(node->get_logger(), "set rate to %f", rate);
    rclcpp::Rate loop_rate(rate);
    //rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    if (node->publishServoConfiguration()) {
        bool debug_mode = node->getNodeConfig().debug_mode;
        rclcpp::Time begin;

        while(rclcpp::ok()){

            if (debug_mode) {
                  begin =node->now();
            }
            node->runOnce();
            
            rclcpp::spin_some(node);
            loop_rate.sleep();

            if (debug_mode){
                std::cout<<(node->now() - begin).seconds() <<std::endl;
            }
        }

    }
  




    rclcpp::spin(node);
    rclcpp::shutdown();
  return 0;
}
