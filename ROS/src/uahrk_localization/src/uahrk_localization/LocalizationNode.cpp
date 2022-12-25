#include "uahrk_localization/LocalizationNode.hpp"
using namespace std::chrono_literals;

LocalizationNode::LocalizationNode(const char *node_name, const char *topic_name)
  : Node(node_name), LocalizationFilter(m,n), node_name(node_name)
{
  // Matrices de prueba para EKF
  LocalizationFilter.H << 1, 1, 1;
  for(int i=0; i<3;++i){LocalizationFilter.Q.diagonal()[i]=100;}
  for(int i=0; i<3;++i){LocalizationFilter.P.diagonal()[i]=0.05;}
  LocalizationFilter.x_hat << 0, 0, 0;

  // Estado inicial de prueba
  LocalizationFilter.x_hat << 1.0, 2.0, 3.0;

  // Publicación cada 100ms
  pose_publisher  = this->create_publisher<geometry_msgs::msg::Pose>(topic_name, 10);
  timer_pub       = this->create_wall_timer(100ms, std::bind(&LocalizationNode::pose_pub_callback, this));
}

void LocalizationNode::pose_pub_callback()
{
  // Prediction 
  LocalizationFilter.prediction();
  // Update
  LocalizationFilter.update();
  // Publicación de ejemplo...
  auto final_pose = geometry_msgs::msg::Pose();
  final_pose.position.x = LocalizationFilter.x_hat(0);
  final_pose.position.y = LocalizationFilter.x_hat(1);
  final_pose.position.z = LocalizationFilter.x_hat(2);
  
  RCLCPP_INFO(this->get_logger(), "Publicando... '%s'", node_name);
  pose_publisher->publish(final_pose);
}

