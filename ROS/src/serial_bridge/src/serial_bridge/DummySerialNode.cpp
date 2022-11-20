#include "serial_bridge/DummySerialNode.hpp"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif


using namespace std::chrono_literals;




geometry_msgs::msg::Pose Pose2dtoPose(const Pose2d &p2d)
{
  geometry_msgs::msg::Pose p;
  p.position.x = p2d.x;
  p.position.y = p2d.y;
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, DEG2RAD(p2d.a));
  tf2::convert(quat_tf.normalize(), p.orientation);
  return p;
}

DummySerialBridgeNode::DummySerialBridgeNode(std::string port_name)
: Node("dummy_serial_bridge_node")
{
  using namespace std::placeholders;
  
  reset_service = this->create_service<uahrk_navigation_msgs::srv::SetPose2d>("set_pose", 
      std::bind(&DummySerialBridgeNode::set_pose, this, _1 ,_2));
  
  
  publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_odom", 10);

  timer_ = this->create_wall_timer(
      500ms, std::bind(&DummySerialBridgeNode::control_cycle, this));

  order_server =  rclcpp_action::create_server<serial_bridge_actions::action::Order>(
    this,
    "serial_bridge_server",
    std::bind(&DummySerialBridgeNode::handle_goal,     this, _1, _2),
    std::bind(&DummySerialBridgeNode::handle_cancel,   this, _1),
    std::bind(&DummySerialBridgeNode::handle_accepted, this, _1));
}

DummySerialBridgeNode::~DummySerialBridgeNode(){}


rclcpp_action::GoalResponse DummySerialBridgeNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Order::Goal> goal)
{
  // Debug Info
  RCLCPP_INFO(this->get_logger(), "Order received");

  // Accept all request
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse  DummySerialBridgeNode::handle_cancel(
  const std::shared_ptr<GoalOrder> goal_handle)
{
  // Accept all  Cancel request
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DummySerialBridgeNode::handle_accepted(const 
  std::shared_ptr<GoalOrder> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(this->get_logger(), "Handling order: Id %s, Arg %d", goal->id.c_str(), goal->arg);
  
  sleep(1);

  auto sim_fn = simulate.find(goal->id);
  if (sim_fn != simulate.end()){
    sim_fn->second(odom,goal->arg);
    auto result   = std::make_shared<Order::Result>();
    goal_handle->succeed(result);
    return;
  }
  else{
    auto result   = std::make_shared<Order::Result>();
    RCLCPP_INFO(this->get_logger(), "Error : Id %s is not an RMI function", goal->id.c_str());
    goal_handle->canceled(result);
    return;
  }
}

void DummySerialBridgeNode::control_cycle(){
  publisher_->publish(Pose2dtoPose(odom));
}

void DummySerialBridgeNode::set_pose(
    const std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Request> request,
    std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Response> response)
{
  odom.x = request->x;
  odom.y = request->y;
  odom.a = request->a;

  RCLCPP_INFO(this->get_logger(), "Setting pose by service to: x: %f y: %f a: %f", request->x
  , request->y, request->a);
}

