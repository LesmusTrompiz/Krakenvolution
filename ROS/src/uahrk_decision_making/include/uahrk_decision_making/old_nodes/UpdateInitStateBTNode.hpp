#pragma once
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
//#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uahrk_navigation_msgs/srv/set_pose2d.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include <unordered_map>
#include "uahrk_decision_making/pose2d.hpp"
#include <chrono>
using namespace std::chrono_literals;

constexpr float n = (0.5 + 0.95)/2;
constexpr float n2 = 2 - n;


std::unordered_map<std::string,Pose2d> blue_spawns = {
    {"1", {0.225          , 3.0 - 0.225        ,  0.0}},
    {"2", {2 - 0.225      , 1.5 + 0.150 + 0.225,  0.0}},
    {"3", {0.225          , 1.5 - 0.150 - 0.225,  0.0}},
    {"4", {0.725          , 0.225              ,  0.0}},
    {"5", {2 - 0.225      , 0.225              ,  0.0}}
};

std::unordered_map<std::string,Pose2d> green_spawns = {
    {"1", {2 - 0.225      , 3.0 - 0.225        , 90.0}},
    {"2", {0.225          , 1.5 + 0.150 + 0.225, 90.0}},
    {"3", {2 - 0.225      , 1.5 - 0.150 - 0.225, 90.0}},
    {"4", {0.225          , 0.225              , 90.0}},
    {"5", {1.275          , 0.225              , 90.0}}
};

class UpdateInitiStateBTNode : public BT::ActionNodeBase
{
    public:
        explicit UpdateInitiStateBTNode(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts()
        {
            return BT::PortsList({BT::OutputPort<std::string>("tree"), BT::OutputPort<std::string>("ally_tree")});
        }
        void halt();


    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time start_time_;
        rclcpp::Client<uahrk_navigation_msgs::srv::SetPose2d>::SharedPtr set_pose_client;
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_playside_param_client;
};


