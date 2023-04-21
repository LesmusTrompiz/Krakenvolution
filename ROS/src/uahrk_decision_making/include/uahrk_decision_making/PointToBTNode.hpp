#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"


class PointTo  : public BT::ActionNodeBase
{
    public:
        explicit PointTo (
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts()
        {
            return BT::PortsList({
                BT::InputPort<double>("x"),
                BT::InputPort<double>("y")
            });
        }
        void halt();

    private:
        rclcpp::Node::SharedPtr node_;
};
