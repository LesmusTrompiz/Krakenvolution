#include "uahrk_navigation/MoveToPoseNode.hpp"

using namespace std::chrono_literals;

extern "C"{
    #include <cassert>
    #include <math.h>
}

MoveToPoseNode::MoveToPoseNode()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MoveToPoseNode::timer_callback, this));
  }

void MoveToPoseNode::timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
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

inline int advance_to_goal(const Point2d &robot, const Point2d &goal){
  /**
   * @brief This function calculates the advance
   * needed by the robot to accomplish the xy
   * goal.
   * 
   * @param robot Actual xy position of the robot
   * @param goal Goal xy position of the goal
   */
  
    // Return the eculidean distance
    return sqrt(((robot.x - goal.x) * (robot.x - goal.x)) + ((robot.y - goal.y) * (robot.y - goal.y)));
}









