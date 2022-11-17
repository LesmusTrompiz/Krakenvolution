#include "uahrk_navigation/RobotTFNode.hpp"

using namespace std::chrono_literals;


RobotTFNode::RobotTFNode() : Node("RobotTFNode"){
    
  // Initialize last_pose ptr
  last_pose = std::make_unique<geometry_msgs::msg::Pose>();

  // Create a tf broadcaster object
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  
  // Subscribe to /robot_odom topic
  subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "robot_odom", 10,
    std::bind(&RobotTFNode::pose_callback, this, std::placeholders::_1));
  
  // Create a timer that will call the control_cycle
  // every 100ms
  timer_ = create_wall_timer(
    50ms, std::bind(&RobotTFNode::control_cycle, this));
}


void 
RobotTFNode::pose_callback(geometry_msgs::msg::Pose::UniquePtr msg)
{
  last_pose = std::move(msg);
}


void RobotTFNode::control_cycle()
{
  geometry_msgs::msg::TransformStamped t;

  // Update the header stamp
  t.header.stamp    = this->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id  = "robot";

  // Update the transform with the last pose
  t.transform.translation.x = last_pose->position.x;
  t.transform.translation.y = last_pose->position.y;
  t.transform.translation.z = last_pose->position.z;
  t.transform.rotation.x    = last_pose->orientation.x;
  t.transform.rotation.y    = last_pose->orientation.y;
  t.transform.rotation.z    = last_pose->orientation.z;
  t.transform.rotation.w    = last_pose->orientation.w;
  
  // Send tht transform
  tf_broadcaster_->sendTransform(t);
}
