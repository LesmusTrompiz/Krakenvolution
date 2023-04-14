#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uahrk_navigation_msgs/action/go_to_pose.hpp"

using GoToPose  = uahrk_navigation_msgs::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;
using Order = serial_bridge_actions::action::Order;


enum state {IDLE, RUNNING};

class GoToPoseBTNode : public BT::ActionNodeBase
{
    public:
        GoToPoseBTNode(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts()
        {
            return BT::PortsList({BT::InputPort<float>("x"), BT::InputPort<float>("y"), BT::InputPort<float>("a") });
        }
        void halt();

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time start_time_;
        state request_state;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr GoToPoseBTNode_pub;
        rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;
        std::shared_future<GoalHandleGoToPose::SharedPtr> request;
        void result_callback(const GoalHandleGoToPose::WrappedResult & result);
        void goal_response_callback(std::shared_future<GoalHandleGoToPose::SharedPtr> future);
        rclcpp_action::ResultCode order_result;
        void sendGoal(float x, float y, float a);
};
