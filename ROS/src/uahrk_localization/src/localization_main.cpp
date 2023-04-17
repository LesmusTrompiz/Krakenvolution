#include "uahrk_localization/LocalizationNode.hpp"
#include "uahrk_localization/RobotSimNode.hpp"
#include "rclcpp/rclcpp.hpp"

float referencia[3] = {200, 0, 5};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>("localization_node", "final_pose_topic"));
  rclcpp::shutdown();
  return 0;
}
