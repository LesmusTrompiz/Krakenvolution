#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "uahrk_navigation/move_utils.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial_bridge_actions/action/order.hpp"
#include "uahrk_navigation_msgs/action/path.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using Order     = serial_bridge_actions::action::Order;
using RequestHandleOrder = rclcpp_action::ClientGoalHandle<Order>;

using Path  = uahrk_navigation_msgs::action::Path;
using GoalHandlePath = rclcpp_action::ServerGoalHandle<Path>;

enum ControlState {IDLE, EXECUTING, NEXT_MOVE};


class MoveToPoseNode : public rclcpp::Node
{
  public:
    MoveToPoseNode();
  private:
    int actual_wp = 0;
    rclcpp_action::Server<Path>::SharedPtr go_to_pose_server;
    rclcpp_action::Client<Order>::SharedPtr    order_client;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
    ControlState state;
    rclcpp_action::ResultCode order_result;
//    std::shared_ptr<const std::shared_ptr<GoalHandlePath>> actual_handle;
    std::shared_ptr<GoalHandlePath> actual_handle;
    std::shared_ptr<geometry_msgs::msg::PoseArray> path_goal;



    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Path::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandlePath> goal_handle);

    void handle_accepted(
      const std::shared_ptr<GoalHandlePath> goal_handle);
    

    void control_cycle();
    void execute(const std::shared_ptr<GoalHandlePath> goal_handle);
    void result_callback(const RequestHandleOrder::WrappedResult & result);
    void goal_response_callback(std::shared_future<RequestHandleOrder::SharedPtr> future);

    void send_order(std::string id, int16_t arg);
    Pose2d get_robot_pose();
};

