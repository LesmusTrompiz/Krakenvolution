#include "uahrk_navigation/MoveToPoseNode.hpp"
using namespace std::chrono_literals;

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


int spin_to_goal(float robot_alfa, float goal_alfa){
  /**
   * @brief This function calculates the spin
   * needed by the robot to accomplish the 
   * angle goal.
   * 
   * @param robot_alfa Actual rotation of the robot
   * in degrees.
   * @param goal_alfa Goal rotation in degrees 
   * 
   */
  int spin = goal_alfa - robot_alfa;
  if      (spin >  180) return spin - 360;
  else if (spin < -180) return spin + 360;
  else                  return spin;
}


