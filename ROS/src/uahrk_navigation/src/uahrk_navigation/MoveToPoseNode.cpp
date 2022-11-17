#include "uahrk_navigation/MoveToPoseNode.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>
using namespace std::chrono_literals;


MoveToPoseNode::MoveToPoseNode()
  : Node("move_to_pose_node")
  {
    using namespace std::placeholders;

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    

    order_client = rclcpp_action::create_client<Order>(this, "serial_bridge_server");
    
    go_to_pose_server = rclcpp_action::create_server<GoToPose>(
      this,
      "move_server",
      std::bind(&MoveToPoseNode::handle_goal, this, _1, _2),
      std::bind(&MoveToPoseNode::handle_cancel, this, _1),
      std::bind(&MoveToPoseNode::handle_accepted, this, _1));  

    // Create a timer that will call the control_cycle
    // every 100ms
    timer_ = create_wall_timer(
      100ms, std::bind(&MoveToPoseNode::control_cycle, this));  
  }

void MoveToPoseNode::control_cycle(){
  auto result = std::make_shared<GoToPose::Result>();
  float dist_precision  = 0.5;
  float angle_precision = 10;

  switch (state)
  {
    case IDLE:
      RCLCPP_INFO(this->get_logger(), "IDLE STATE");
      if (actual_handle != nullptr && actual_handle->is_active()){
        state = NEXT_MOVE;
      }
    break;
    case NEXT_MOVE:
      try {
        RCLCPP_INFO(this->get_logger(), "NEXT MOVE STATE");
        auto robot_pose = get_robot_pose();
        Pose2d goal_pose{actual_handle->get_goal()->pose.pose};
        if (robot_in_goal(robot_pose, goal_pose, dist_precision, angle_precision)){
          actual_handle->succeed(result);
          state = IDLE;
        }
        else{
          auto move = calculate_move(robot_pose, goal_pose, dist_precision, angle_precision);
          send_order(std::get<0>(move),std::get<1>(move));
          state = EXECUTING;
        }
      }
      catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
        actual_handle->abort(result);
        state = IDLE;
        return;
      }
      catch (const std::exception &exc)
      {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << exc.what();
        RCLCPP_ERROR(get_logger(), "Unknown Exception not found: %s", exc.what());
        actual_handle->abort(result);
        state = IDLE;
        return;
      }
    break;
    case EXECUTING:
      // Check out if the order to serial
      // Node has finished
      RCLCPP_INFO(this->get_logger(), "EXECUTING");
      switch (order_result) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          state = NEXT_MOVE;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          actual_handle->abort(result);
          state = IDLE;
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          actual_handle->abort(result);
          state = IDLE;
          break;
        default:
          break;
      }
    break;
  default:
    RCLCPP_ERROR(get_logger(), "Unknown state %i in the control cycle loop", state);
    state = IDLE;
    break;
  }
}


/*
  Client Functions:
*/

void MoveToPoseNode::send_order(std::string id, int16_t arg)
{
  auto send_goal_options = rclcpp_action::Client<Order>::SendGoalOptions();
  auto goal_msg = Order::Goal();
  order_result = rclcpp_action::ResultCode::UNKNOWN;

  if (order_client->wait_for_action_server(300ms)){
    throw std::logic_error("Could not contact the Serial Bridge Server. Throwing an exception");
  }

  send_goal_options.result_callback = std::bind(&MoveToPoseNode::result_callback, this, std::placeholders::_1);

  goal_msg.id  = id;
  goal_msg.arg = arg;
  RCLCPP_INFO(this->get_logger(), "Sending msg %s %i", id.c_str(), arg);

  order_client->async_send_goal(goal_msg, send_goal_options);
}

void MoveToPoseNode::result_callback(const RequestHandleOrder::WrappedResult & result)
{
  order_result = result.code;
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
  // Update the actual handle variable an
  // acomplish the goal in control_cycle
  actual_handle = goal_handle;
}

Pose2d MoveToPoseNode::get_robot_pose(){
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped robot_tf = tf_buffer_->lookupTransform(
      "map", "robot", now, 100ms);
    return {robot_tf};
}


