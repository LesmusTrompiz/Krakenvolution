#include <string>
#include <iostream>
#include "uahrk_decision_making/HelloNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
Hello::Hello( const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
    hello_pub = node_->create_publisher<std_msgs::msg::String>("pub_hello",10);
}

BT::NodeStatus Hello::tick(){
    std_msgs::msg::String msg;
    // msg.data = "hello";
    getInput("print", msg.data);
    hello_pub->publish(msg);
    cnt++;
    if(cnt==100){
        cnt= 0;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void Hello::halt(){
    return;
}
 // namespace br2_bt_bumpgo
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<Hello>("Hello");
}
