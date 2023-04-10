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
#include <chrono>
#include "std_srvs/srv/trigger.hpp"
using namespace std::chrono_literals;




class WaitStartSignalBTNode : public BT::ActionNodeBase
{
    public:
        explicit WaitStartSignalBTNode(
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
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_condition_client;
};


