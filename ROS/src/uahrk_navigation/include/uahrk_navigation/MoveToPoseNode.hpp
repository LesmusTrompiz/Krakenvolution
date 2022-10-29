#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "uahrk_navigation/point2d.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MoveToPoseNode : public rclcpp::Node
{
  public:
    MoveToPoseNode();
  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    
};

int spin_to_goal(float robot_alfa, float goal_alfa);

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




