#include "uahrk_navigation/RobotTFNode.hpp"
#include <iostream>
int main(int argc, char * argv[])
{
  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotTFNode>());
  rclcpp::shutdown();
  return 0;
}