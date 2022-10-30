#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "uahrk_navigation/point2d.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial_bridge_actions/action/order.hpp"
#include "uahrk_navigation_msgs/action/go_to_pose.hpp"

//using go_to_pose = uahrk_navigation_msgs::action::GoToPose;
//using Fibonacci = action_tutorials_interfaces::action::Fibonacci;


using Order     = serial_bridge_actions::action::Order;
using GoToPose  = uahrk_navigation_msgs::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

class MoveToPoseNode : public rclcpp::Node
{
  
  public:
    MoveToPoseNode();
  private:
    rclcpp_action::Server<GoToPose>::SharedPtr go_to_pose_server;
    rclcpp_action::Client<Order>::SharedPtr    order_client;
    

    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPose::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleGoToPose> goal_handle);

    void handle_accepted(
      const std::shared_ptr<GoalHandleGoToPose> goal_handle);
    
    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
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




