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
                BT::OutputPort<std::string>("x"),
                BT::OutputPort<std::string>("y"),
                BT::OutputPort<std::string>("theta")
            });
        }
        void halt();

    private:
        rclcpp::Node::SharedPtr node_;
};
