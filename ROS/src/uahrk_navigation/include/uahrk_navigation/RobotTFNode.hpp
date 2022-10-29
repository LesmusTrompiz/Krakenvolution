#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class RobotTFNode : public rclcpp::Node
{
public:  
  RobotTFNode();

private:
  void pose_callback(geometry_msgs::msg::Pose::UniquePtr msg);
  void control_cycle();
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  geometry_msgs::msg::Pose::UniquePtr last_pose;

};






