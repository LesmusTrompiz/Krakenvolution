#include "uahrk_navigation/PathFindingNode.hpp"

int main(int argc, char * argv[])
{
  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFindingNode>());
  rclcpp::shutdown();
  return 0;
}