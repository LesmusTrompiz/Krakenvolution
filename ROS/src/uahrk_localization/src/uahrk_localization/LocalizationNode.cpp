#include "uahrk_localization/LocalizationNode.hpp"
#include <cmath>


using namespace std::chrono_literals;

inline float DEG2RAD(float x){
  return x * M_PI /180.0;
}
LocalizationNode::LocalizationNode(const char *node_name, const char *final_pose_topic)
  : Node(node_name), LocalizationFilter(1,3)
{
  // Matrices de prueba para EKF
  LocalizationFilter.A << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  LocalizationFilter.H << 1, 1, 1;
  
  std::cout << LocalizationFilter.A ;
  for(int i=0; i<3;++i){LocalizationFilter.Q.diagonal()[i]=100;}
  for(int i=0; i<3;++i){LocalizationFilter.P.diagonal()[i]=0.05;}
  
  
  // Estado inicial de prueba
  LocalizationFilter.x_hat << 0.0, 0.0, 0.0;

  // Publicación cada 100ms
  final_pose_publisher  = this->create_publisher<geometry_msgs::msg::PoseStamped>(final_pose_topic, 10);
  timer                 = this->create_wall_timer(100ms, std::bind(&LocalizationNode::pose_pub_callback, this));

  // Toma de datos por topic
  vel_sub             = this->create_subscription<geometry_msgs::msg::TwistStamped>("robot_vel", 10, std::bind(&LocalizationNode::vel_callback,      this, std::placeholders::_1));
  odom_sub              = this->create_subscription<geometry_msgs::msg::PoseStamped>("odom_pose", 10, std::bind(&LocalizationNode::odom_callback,      this, std::placeholders::_1));
  lidar_sub             = this->create_subscription<geometry_msgs::msg::PoseStamped>("lidar_pose", 10, std::bind(&LocalizationNode::lidar_callback,     this, std::placeholders::_1));
  camera_sub            = this->create_subscription<geometry_msgs::msg::PoseStamped>("camera_pose", 10, std::bind(&LocalizationNode::camera_callback,  this, std::placeholders::_1));


  // Initialize the transform broadcasster
  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void doTransform(geometry_msgs::msg::PoseStamped::UniquePtr &t_in, const geometry_msgs::msg::TransformStamped& transform){
  KDL::Vector v(t_in->pose.position.x, t_in->pose.position.y, t_in->pose.position.z);
  KDL::Rotation r = KDL::Rotation::Quaternion(t_in->pose.orientation.x, t_in->pose.orientation.y, t_in->pose.orientation.z, t_in->pose.orientation.w);
  KDL::Frame v_out = tf2::gmTransformToKDL(transform) * KDL::Frame(r, v);

  t_in->pose.position.x = v_out.p[0];
  t_in->pose.position.y = v_out.p[1];
  t_in->pose.position.z = v_out.p[2];
  v_out.M.GetQuaternion(t_in->pose.orientation.x, t_in->pose.orientation.y, t_in->pose.orientation.z, t_in->pose.orientation.w);
}


void LocalizationNode::odom_callback(geometry_msgs::msg::PoseStamped::UniquePtr odom_msg)
{
  doTransform(last_odom_pose, tf_robot_odom);
  last_odom_pose = std::move(odom_msg);
  return;
}

void LocalizationNode::lidar_callback(geometry_msgs::msg::PoseStamped::UniquePtr lidar_msg)
{
  last_lidar_pose = std::move(lidar_msg);
  return;
}

void LocalizationNode::camera_callback(geometry_msgs::msg::PoseStamped::UniquePtr camera_msg){
  last_camera_pose = std::move(camera_msg);
  return;
}

void LocalizationNode::vel_callback(geometry_msgs::msg::TwistStamped::UniquePtr twist_msg){
  last_vel = std::move(twist_msg);
  return;
}


void LocalizationNode::pose_pub_callback()
{
  // Discard measuares if they 
  // stale out

  // Prediction 
  LocalizationFilter.prediction();
  
  // Update for each measure that is not a 
  // null pointer
  LocalizationFilter.update();
  
  // Publicamos la información
  geometry_msgs::msg::TransformStamped t;

  // Read message content and assign it to
  // corresponding tf variables
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id  = "robot";

  // Turtle only exists in 2D, thus we get x and y translation
  // coordinates from the message and set the z coordinate to 0
  t.transform.translation.x = LocalizationFilter.x_hat(0);
  t.transform.translation.y = LocalizationFilter.x_hat(1);
  t.transform.translation.z = 0.0;

  // For the same reason, turtle can only rotate around one axis
  // and this why we set rotation in x and y to 0 and obtain
  // rotation in z axis from the message
  tf2::Quaternion q;
  q.setRPY(0, 0, DEG2RAD(LocalizationFilter.x_hat(2)));
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
}

