#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/int16.hpp"
#include "rclcpp/rclcpp.hpp"


class PubPointsBTNode : public BT::ActionNodeBase
{
    public:
        explicit PubPointsBTNode(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts()
        {
            return BT::PortsList({BT::InputPort<int>("points")});
        }
        void halt();

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time start_time_;
        int cnt = 0;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_points;
};
