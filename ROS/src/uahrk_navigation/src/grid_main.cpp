#include "uahrk_navigation/GridNode.hpp"
#include <iostream>
int main(int argc, char * argv[])
{
  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridNode>());
  rclcpp::shutdown();
  return 0;
}