#include "uahrk_localization/LocalizationNode.hpp"
#include "uahrk_localization/RobotSimNode.hpp"
#include "rclcpp/rclcpp.hpp"

float referencia[3] = {200, 0, 5};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  
  rclcpp::Node::SharedPtr localization_node = std::make_shared<LocalizationNode>("localization_node", "final_pose_topic");
  rclcpp::Node::SharedPtr simulation_node   = std::make_shared<RobotSimNode>("simulation_node", "odom_pose", 300, referencia);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(localization_node);
  executor.add_node(simulation_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
