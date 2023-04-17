#include <iostream>
#include "uahrk_decision_making/PubPointsBTNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

PubPointsBTNode::PubPointsBTNode( const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
    pub_points = node_->create_publisher<std_msgs::msg::Int16>("points",10);
}

BT::NodeStatus PubPointsBTNode::tick(){
    std_msgs::msg::Int16 msg;
    int a;
    getInput("points", a);
    msg.data = a;
    pub_points->publish(msg);
    return BT::NodeStatus::SUCCESS;
}

void PubPointsBTNode::halt(){
    return;
}
 // namespace br2_bt_bumpgo
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<PubPointsBTNode>("PubPoints");
}
