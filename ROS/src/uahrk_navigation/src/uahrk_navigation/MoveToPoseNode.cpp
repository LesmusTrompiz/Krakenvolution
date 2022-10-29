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
    go_to_pose_server = rclcpp_action::create_server<GoToPose>(
      this,
      "move_server",
      std::bind(&MoveToPoseNode::handle_goal, this, _1, _2),
      std::bind(&MoveToPoseNode::handle_cancel, this, _1),
      std::bind(&MoveToPoseNode::handle_accepted, this, _1));
  }

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
  auto result = std::make_shared<GoToPose::Result>();

  result->result.data = true;
  goal_handle->succeed(result);
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  //std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
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











