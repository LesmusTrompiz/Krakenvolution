#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"


class FindBasket : public BT::ActionNodeBase
{
    public:
        explicit FindBasket(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts()
        {
            return BT::PortsList({
                BT::OutputPort<double>("x"),
                BT::OutputPort<double>("y"),
                BT::OutputPort<double>("theta")
            });
        }
        void halt();

    private:
        rclcpp::Node::SharedPtr node_;
};
