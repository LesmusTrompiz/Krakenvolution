#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"


class Hello : public BT::ActionNodeBase
{
    public:
        explicit Hello(
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
        rclcpp::Time start_time_;
        int cnt = 0;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hello_pub;
};
