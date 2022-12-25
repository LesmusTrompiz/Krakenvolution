#include "uahrk_localization/LocalizationNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::Node::SharedPtr localization_node = std::make_shared<LocalizationNode>("st_node", "st_topic");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(localization_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
