#include "uahrk_localization/LocalizationNode.hpp"
using namespace std::chrono_literals;

LocalizationNode::LocalizationNode(const char *node_name, const char *final_pose_topic)
  : Node(node_name), LocalizationFilter(1,3)
{
  // Matrices de prueba para EKF
  LocalizationFilter.H << 1, 1, 1;
  for(int i=0; i<3;++i){LocalizationFilter.Q.diagonal()[i]=100;}
  for(int i=0; i<3;++i){LocalizationFilter.P.diagonal()[i]=0.05;}
  // Estado inicial de prueba
  LocalizationFilter.x_hat << 1.0, 2.0, 3.0;

  // Publicación cada 100ms
  final_pose_publisher  = this->create_publisher<geometry_msgs::msg::PoseStamped>(final_pose_topic, 10);
  timer_pub             = this->create_wall_timer(100ms, std::bind(&LocalizationNode::pose_pub_callback, this));

  // Toma de datos por topic
  odom_sub              = this->create_subscription<geometry_msgs::msg::PoseStamped>("odom_pose", 10,   std::bind(&LocalizationNode::odom_callback,      this, std::placeholders::_1));
  lidar_sub             = this->create_subscription<geometry_msgs::msg::PoseStamped>("lidar_pose", 10,  std::bind(&LocalizationNode::lidar_callback,     this, std::placeholders::_1));
  camera_sub            = this->create_subscription<geometry_msgs::msg::PoseStamped>("camera_pose", 10,   std::bind(&LocalizationNode::camera_callback,  this, std::placeholders::_1));
}


void LocalizationNode::odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr odom_msg)
{
  // // @todo: Conteo de fuentes de información.
  //   = odom_msg->data.header.stamp.secs()
  // // Obtención de los datos:
  //   = odom_msg->data.pose.position.x; 
  //   = odom_msg->data.pose.position.y;
  //   = odom_msg->data.pose.position.z;
  return;
}

void LocalizationNode::lidar_callback(const geometry_msgs::msg::PoseStamped::SharedPtr lidar_msg)
{
  // // @todo: Conteo de fuentes de información.
  //   = lidar_msg->data.header.stamp.secs()
  // // Obtención de los datos:
  //   = lidar_msg->data.pose.position.x; 
  //   = lidar_msg->data.pose.position.y;
  //   = lidar_msg->data.pose.position.z; 
  return;
}

void LocalizationNode::camera_callback(const geometry_msgs::msg::PoseStamped::SharedPtr camera_msg)
{
  // // @todo: Conteo de fuentes de información.
  //   = camera_msg->data.header.stamp.secs()
  // // Obtención de los datos:
  //   = camera_msg->data.pose.position.x; 
  //   = camera_msg->data.pose.position.y;
  //   = camera_msg->data.pose.position.z; 
  return;
}


void LocalizationNode::pose_pub_callback()
{
  // Prediction 
  LocalizationFilter.prediction();
  // Update
  LocalizationFilter.update();
  // Publicación de ejemplo...
  auto final_pose = geometry_msgs::msg::PoseStamped();
  final_pose.pose.position.x = LocalizationFilter.x_hat(0);
  final_pose.pose.position.y = LocalizationFilter.x_hat(1);
  final_pose.pose.position.z = LocalizationFilter.x_hat(2);
  // Publicamos la información
  final_pose_publisher->publish(final_pose);
}

