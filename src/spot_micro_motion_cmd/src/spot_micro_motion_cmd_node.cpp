// Node file to create object and initialising the ROS node
#include "spot_micro_motion_cmd.h" 
#include <iostream>


int main(int argc, char** argv) {
  /* initialising the ROS node creating node handle
  for regestring it to the master and then private node handle to
  handle the parameters */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SpotMicroMotionCmd>();
 
    float duration = node->getNodeConfig().dt;
    float rate = 1.0/duration;
   
    
    rclcpp::Rate loop_rate(rate);


    if (node->publishServoConfiguration()) {
        bool debug_mode = node->getNodeConfig().debug_mode;
        rclcpp::Time begin;
        while(rclcpp::ok()){

            

            if (debug_mode) {
                  begin =node->now();
            }

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