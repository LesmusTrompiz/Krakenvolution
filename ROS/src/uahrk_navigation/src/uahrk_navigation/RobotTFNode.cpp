#include "uahrk_navigation/RobotTFNode.hpp"


RobotTFNode::RobotTFNode():
  Node("static_RobotTFNode"){
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms();
}


void RobotTFNode::make_transforms()
{
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id =  "robot";

  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_static_broadcaster_->sendTransform(t);
}
