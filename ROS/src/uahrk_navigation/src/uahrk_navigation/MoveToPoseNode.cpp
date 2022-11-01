#include "uahrk_navigation/MoveToPoseNode.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using namespace std::chrono_literals;

extern "C"{
    #include <cassert>
    #include <math.h>
}

MoveToPoseNode::MoveToPoseNode()
  : Node("move_to_pose_node")
  {
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    using namespace std::placeholders;
    
    order_client = rclcpp_action::create_client<Order>(this, "serial_bridge_server");
    
    
    go_to_pose_server = rclcpp_action::create_server<GoToPose>(
      this,
      "move_server",
      std::bind(&MoveToPoseNode::handle_goal, this, _1, _2),
      std::bind(&MoveToPoseNode::handle_cancel, this, _1),
      std::bind(&MoveToPoseNode::handle_accepted, this, _1));    
  }

/*
  Client Functions:
*/

void MoveToPoseNode::send_order()
{
  auto send_goal_options = rclcpp_action::Client<Order>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&MoveToPoseNode::result_callback, this, std::placeholders::_1);
  auto goal_msg = Order::Goal();
  goal_msg.id  = "Prueba";
  goal_msg.arg = 1;
  order_client->async_send_goal(goal_msg, send_goal_options);
}

void MoveToPoseNode::result_callback(const RequestHandleOrder::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  std::stringstream ss;
  ss << "Result received: " << result.result->ret;
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

/*
  SERVER FUNCS:

*/


rclcpp_action::GoalResponse MoveToPoseNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToPose::Goal> goal){

    RCLCPP_INFO(this->get_logger(), 
      "Received goal request with order x: %f y: %f  ",
      goal->pose.pose.position.x,
      goal->pose.pose.position.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse MoveToPoseNode::handle_cancel(
  const std::shared_ptr<GoalHandleGoToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveToPoseNode::handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
{
  using namespace std::placeholders;

  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&MoveToPoseNode::execute, this, _1), goal_handle}.detach();
}


void MoveToPoseNode::execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  send_order();
  auto result = std::make_shared<GoToPose::Result>();
  result->result.data = true;
  RCLCPP_INFO(this->get_logger(), "Goal executed");
  goal_handle->succeed(result);
}




int spin_to_goal(const float robot_alfa, const float goal_alfa){
  /**
   * @brief This function calculates the spin
   * needed by the robot to accomplish the 
   * angle goal.
   * 
   * @param robot_alfa Actual rotation of the robot
   * in degrees, the expected range is from 
   * -180 to 180 (ROS STANDARD).
   * @param goal_alfa Goal rotation in degrees 
   * the expected range is from  -180 to 180
   * (ROS STANDARD).
   */
  
  // Check that arguments are in the expected range
  assert(robot_alfa <=  180);
  assert(robot_alfa >= -180);
  assert(goal_alfa  <=  180);
  assert(goal_alfa  >= -180);


  int spin = goal_alfa - robot_alfa;
  if      (spin >  180) return spin - 360;
  else if (spin < -180) return spin + 360;
  else                  return spin;
}











