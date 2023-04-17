#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "serial_bridge_actions/action/order.hpp"

//#define Order serial_bridge_actions::action::Order
using Order = serial_bridge_actions::action::Order;
// using Order = serial_bridge_actions::action::Order;
using RequestHandleOrder = rclcpp_action::ClientGoalHandle<Order>;

class Caller : public BT::ActionNodeBase
{
    public:
        explicit Caller(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts()
        {
            return BT::PortsList({BT::InputPort<std::string>("device"), BT::InputPort<std::string>("id"), BT::InputPort<int>("arg") });
        }
        void halt();

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time start_time_;
        int cnt = 0;
        rclcpp_action::Client<Order>::SharedPtr    order_client;
        rclcpp_action::ResultCode order_result;
        enum State {
          SENDING_COMMAND,
          WAITING_RESPONSE
        } current_state;

        void invoke_callback(const RequestHandleOrder::WrappedResult & result);
        // void goal_response_callback(std::shared_future<RequestHandleOrder::SharedPtr> future);
};
