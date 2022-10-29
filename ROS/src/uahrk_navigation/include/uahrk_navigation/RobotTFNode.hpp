#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class RobotTFNode : public rclcpp::Node
{
public:  
  RobotTFNode();

private:
  void make_transforms();
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

};




