#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"


class Timer : public BT::ConditionNode
{
    public:
        explicit Timer(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts()
        {
            return BT::PortsList({BT::InputPort<std::string>("time")});
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time start_time_;

        uint64_t game_start_time = 0;
        uint64_t timeout = 0;
};
