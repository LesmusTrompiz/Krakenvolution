#include <string>
#include <iostream>
#include "uahrk_decision_making/CallerBTNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
Caller::Caller( const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
    order_client = rclcpp_action::create_client<Order>(this, "serial_bridge_server");
}

BT::NodeStatus Caller::tick(){
    switch (current_state) {
      case SENDING_COMMAND: {
        std::string device;
        std::string id;
        int arg;
        getInput("device", device);
        getInput("id", id);
        getInput("arg", arg);

        auto send_goal_options = rclcpp_action::Client<Order>::SendGoalOptions();
        auto goal_msg = Order::Goal();
        order_result = rclcpp_action::ResultCode::UNKNOWN;
        
        current_state = WAITING_RESPONSE;
        break;
      }
      case WAITING_RESPONSE: {

      switch (order_result){
        case rclcpp_action::ResultCode::SUCCEEDED:
          return BT::NodeStatus::SUCCESS;
          break;
        case rclcpp_action::ResultCode::ABORTED:
        case rclcpp_action::ResultCode::CANCELED:
          return BT::NodeStatus::FAILURE;
          break;
      }


        break;
      }
    }

    // if (!order_client->wait_for_action_server(300ms)){
    //   throw std::logic_error("Could not contact the Serial Bridge Server. Throwing an exception");
    // }

    send_goal_options.result_callback = std::bind(&MoveToPoseNode::result_callback, this, std::placeholders::_1);
    send_goal_options.goal_response_callback = std::bind(&MoveToPoseNode::goal_response_callback, this, std::placeholders::_1);
    goal_msg.id  = id;
    goal_msg.arg = arg;
    RCLCPP_INFO(this->get_logger(), "Sending msg %s %i", id.c_str(), arg);
    order_client->async_send_goal(goal_msg, send_goal_options);
    return BT::NodeStatus::RUNNING;
}

void Caller::halt(){
    return;
}

void Caller::invoke_callback(const RequestHandleOrder::WrappedResult & result) {
  order_result = result.code;
}
 // namespace br2_bt_bumpgo
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<Caller>("Caller");
}
