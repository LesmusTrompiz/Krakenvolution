#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial_bridge_actions/action/order.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "uahrk_navigation_msgs/srv/set_pose2d.hpp"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif


using namespace std::chrono_literals;

inline float DEG2RAD(const float deg)
{
  return deg* M_PI / 180;
}

struct Pose2d
{
    float x;
    float y;
    float a;

    Pose2d()                             : x{0.0} , y{0.0} , a{0.0} {};
    Pose2d(float _x, float _y, float _a) : x{_x}  , y{_y}  , a{_a}  {};

    friend inline bool operator==(const Pose2d& lhs, const Pose2d& rhs) { 
        return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.a == rhs.a); 
    }

    friend inline bool operator!=(const Pose2d& lhs, const Pose2d& rhs) {
        return !(lhs == rhs);
    }
};

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

void simulate_advance(Pose2d &p,int distance)
{
  float d = (float)distance/1000;
  p.x += d * cos(DEG2RAD(p.a));
  p.y += d * sin(DEG2RAD(p.a));
}

void simulate_spin(Pose2d &p,int spin)
{
  p.a += spin;
  if (p.a >  180) p.a -= 360;
  if (p.a < -180) p.a += 360;
}

class DummySerialBridgeNode : public rclcpp::Node
{
  public:
    using Order     = serial_bridge_actions::action::Order;
    using GoalOrder = rclcpp_action::ServerGoalHandle<Order>;
    DummySerialBridgeNode(std::string port_name);
    ~DummySerialBridgeNode();
    rclcpp_action::GoalResponse     handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const Order::Goal> goal);
    rclcpp_action::CancelResponse   handle_cancel(const std::shared_ptr<GoalOrder> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalOrder> goal_handle);

  private:
    void control_cycle();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<serial_bridge_actions::action::Order>::SharedPtr order_server;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Service<uahrk_navigation_msgs::srv::SetPose2d>::SharedPtr reset_service;
    void set_pose(const std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Request> request,
             std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Response> response);

    Pose2d odom;
};

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

  if (goal->id=="advance")  simulate_advance(odom,goal->arg);
  else if(goal->id=="spin") simulate_spin(odom,goal->arg);
  
  auto result   = std::make_shared<Order::Result>();
  //result.return = rmi.invoque(goal->id,goal->arg);
  goal_handle->succeed(result);
  return;
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


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySerialBridgeNode>("hola"));
  rclcpp::shutdown();
  return 0;
}







