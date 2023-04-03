#pragma once
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
//#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include <unordered_map>
#include <chrono>
#include "std_srvs/srv/empty.hpp"
using namespace std::chrono_literals;




class ResetParamsBTNode : public BT::ActionNodeBase
{
    public:
        explicit ResetParamsBTNode(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts()
        {
            return BT::PortsList({});
        }
        void halt();

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_condition_client;
};


