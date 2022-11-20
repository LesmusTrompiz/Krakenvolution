#include "serial_bridge/SerialBridgeNode.hpp"


SerialBridgeNode::SerialBridgeNode(std::string port_name)
: Node("serial_bridge_node")
{
  using namespace std::placeholders;
  
  order_server =  rclcpp_action::create_server<serial_bridge_actions::action::Order>(
    this,
    "serial_bridge_server",
    std::bind(&SerialBridgeNode::handle_goal,     this, _1, _2),
    std::bind(&SerialBridgeNode::handle_cancel,   this, _1),
    std::bind(&SerialBridgeNode::handle_accepted, this, _1));
    //get_parameter("port_name", port_name);

    //?protocol.hook_serial_port(port);
}

SerialBridgeNode::~SerialBridgeNode(){}



rclcpp_action::GoalResponse SerialBridgeNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Order::Goal> goal)
{
  // Debug Info
  RCLCPP_INFO(this->get_logger(), "Order received");

  // Accept all request
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse  SerialBridgeNode::handle_cancel(
  const std::shared_ptr<GoalOrder> goal_handle)
{
  // Accept all  Cancel request
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SerialBridgeNode::handle_accepted(const 
  std::shared_ptr<GoalOrder> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(this->get_logger(), "Handling order: Id %s, Arg %d", goal->id.c_str(), goal->arg);
  auto result   = std::make_shared<Order::Result>();
  //result.return = rmi.invoque(goal->id,goal->arg);
  goal_handle->succeed(result);
  return;
}
