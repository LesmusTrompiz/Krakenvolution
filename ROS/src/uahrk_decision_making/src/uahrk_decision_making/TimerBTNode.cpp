#include <string>
#include <iostream>
#include "uahrk_decision_making/TimerBTNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>

using namespace std::chrono_literals;
Timer::Timer( const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
    : BT::ConditionNode(xml_tag_name, conf){
    config().blackboard->get("node", node_);
    config().blackboard->get("game_start_timestamp", game_start_time);
    getInput("time", timeout);
}

BT::NodeStatus Timer::tick(){
    auto start_time = std::chrono::high_resolution_clock::now();
    auto t = std::chrono::time_point_cast<std::chrono::milliseconds>(start_time).time_since_epoch().count();
    std::cout << "Time: " << t << "/" << (game_start_time) << " thefuck\n";
    if (t >= (timeout + game_start_time)) {
        std::cout << "Timer finished\n";
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

 // namespace br2_bt_bumpgo
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<Timer>("Timer");
}
