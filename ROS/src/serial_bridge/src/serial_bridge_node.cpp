#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial_bridge/action/order.hpp"
#include "protocol.hpp"

using namespace std::chrono_literals;


class SerialBridgeNode : public rclcpp::Node
{
  public:
    using Order     = serial_bridge::action::Order;
    using GoalOrder = rclcpp_action::ServerGoalHandle<Order>;
    SerialBridgeNode(std::string port_name);
    ~SerialBridgeNode();
    rclcpp_action::GoalResponse     handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const Order::Goal> goal);
    rclcpp_action::CancelResponse   handle_cancel(const std::shared_ptr<GoalOrder> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalOrder> goal_handle);

    //?api::Instance protocol;

  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp_action::Server<serial_bridge::action::Order>::SharedPtr order_server;
};

SerialBridgeNode::SerialBridgeNode(std::string port_name)
: Node("serial_bridge_node"), count_(0),vprotocol()
{
  using namespace std::placeholders;
  
  order_server =  rclcpp_action::create_server<serial_bridge::action::Order>(
    this,
    "serial_bridge_server",
    std::bind(&SerialBridgeNode::handle_goal,     this, _1, _2),
    std::bind(&SerialBridgeNode::handle_cancel,   this, _1),
    std::bind(&SerialBridgeNode::handle_accepted, this, _1));
    get_parameter("port_name", port_name);

    //?protocol.hook_serial_port(port);
}

SerialBridgeNode::~SerialBridgeNode(){}


void SerialBridgeNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}


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



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>("hola"));
  rclcpp::shutdown();
  return 0;
}


