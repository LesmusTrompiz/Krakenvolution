#include "uahrk_decision_making/PointToBTNode.hpp"

#include <string>
#include <iostream>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

PointTo::PointTo( const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
}

BT::NodeStatus PointTo::tick(){
    return BT::NodeStatus::RUNNING;
}

void PointTo::halt(){
    return;
}
 // namespace br2_bt_bumpgo
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<PointTo>("PointTo");
}
