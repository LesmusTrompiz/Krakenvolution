#include "uahrk_navigation/MoveToPoseNode.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>
using namespace std::chrono_literals;

// Utils:
ControlState analize_order(rclcpp_action::ResultCode order_result){
  switch (order_result){
    case rclcpp_action::ResultCode::SUCCEEDED:
      return NEXT_MOVE;

    case rclcpp_action::ResultCode::ABORTED:
      return IDLE;
    
    case rclcpp_action::ResultCode::CANCELED:
      return IDLE;

    default:
      return WAITING_RESULT;
  }
}

bool new_goal(std::shared_ptr<GoalHandlePath> actual_handle){
  return (actual_handle != nullptr && actual_handle->is_active());
}

bool last_wp(std::vector<geometry_msgs::msg::Pose> poses, int actual_wp){
  return (actual_wp == (poses.size() -1));
}


MoveToPoseNode::MoveToPoseNode()
  : Node("move_to_pose_node")
  {
    using namespace std::placeholders;
    pub_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("robot_vel", 10);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    order_client = rclcpp_action::create_client<Order>(this, "serial_bridge_server");
    set_odom_tf_client = create_client<uahrk_navigation_msgs::srv::SetPose2d>("reset_odom");
    go_to_pose_server = rclcpp_action::create_server<Path>(
      this,
      "move_server",
      std::bind(&MoveToPoseNode::handle_goal, this, _1, _2),
      std::bind(&MoveToPoseNode::handle_cancel, this, _1),
      std::bind(&MoveToPoseNode::handle_accepted, this, _1));  

    // Create a timer that will call the control_cycle
    // every 100ms
    timer_ = create_wall_timer(
      100ms, std::bind(&MoveToPoseNode::control_cycle, this));  
    RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR -> IDLE STATE");
    actual_vel.header.frame_id = "odom";
    actual_vel.header.stamp = this->get_clock()->now();

  }

void MoveToPoseNode::control_cycle(){
  auto result = std::make_shared<Path::Result>();

  // Publish Actual Vel
  pub_vel->publish(actual_vel);


  switch(state)
  {
    case IDLE:
      if (new_goal(actual_handle)){
        state = NEXT_MOVE;
        RCLCPP_INFO(this->get_logger(), "IDLE -> NEXT MOVE");
      }
      return;
    case NEXT_MOVE:
      try{
        auto robot_pose = get_robot_pose();
        Pose2d goal_pose{path_goal->poses[actual_wp]};
        if(last_wp(path_goal->poses, actual_wp) && in_wp_and_angle(robot_pose,goal_pose)){
          actual_handle->succeed(result);
          state = IDLE;
          return;
        }
        else if(!last_wp(path_goal->poses, actual_wp) && in_wp(robot_pose,goal_pose)){
          actual_wp++;
          goal_pose = {path_goal->poses[actual_wp]};
        }
        auto move = calculate_move(robot_pose, goal_pose, dist_precision, angle_precision);
        auto id = std::get<0>(move);
        auto arg = std::get<1>(move);

        // Update actual vel
        actual_vel.twist = geometry_msgs::msg::Twist();

        if(id == "turn"){
          if(arg > 0){
            actual_vel.twist.angular.z = 1;
          }else{
            actual_vel.twist.angular.z = -1;
          }
        }
        else if(id == "advance"){
          if(arg > 0){
            actual_vel.twist.linear.x = 1;
          }else{
            actual_vel.twist.linear.x = -1;
          }
        }

        send_order(id,arg);
        state = WAITING_RESULT;
        return;
      }
      catch (tf2::TransformException & ex){
        RCLCPP_WARN(get_logger(), "Robot transform not found: %s", ex.what());
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
    case WAITING_RESULT:
      // Check out if the order to serial Node has finished
      state = analize_order(order_result);

      // If we pass to IDLE we must abort the actual goal.
      if(state == IDLE){
        actual_handle->abort(result);
      }
      return;
  default:
    RCLCPP_ERROR(get_logger(), "Unknown state %i in the control cycle loop", state);
    RCLCPP_INFO(this->get_logger(), "UNKNOWN -> IDLE");
    state = IDLE;
    break;
  }
}


/*
  Client Functions:
*/


void MoveToPoseNode::goal_response_callback(std::shared_future<RequestHandleOrder::SharedPtr> future)
{
  auto goal_handle = future.get();

  if (!goal_handle) {
    actual_vel.header.stamp = this->get_clock()->now();
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    actual_vel.twist = geometry_msgs::msg::Twist();

    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void MoveToPoseNode::send_order(std::string id, int16_t arg)
{
  auto send_goal_options = rclcpp_action::Client<Order>::SendGoalOptions();
  auto goal_msg = Order::Goal();
  order_result = rclcpp_action::ResultCode::UNKNOWN;

  if (!order_client->wait_for_action_server(300ms)){
    throw std::logic_error("Could not contact the Serial Bridge Server. Throwing an exception");
  }

  send_goal_options.result_callback = std::bind(&MoveToPoseNode::result_callback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback = std::bind(&MoveToPoseNode::goal_response_callback, this, std::placeholders::_1);
  goal_msg.device  = "traction";
  goal_msg.id  = id;
  goal_msg.arg = arg;
  RCLCPP_INFO(this->get_logger(), "Sending msg %s %i", id.c_str(), arg);
  order_client->async_send_goal(goal_msg, send_goal_options);
}

void MoveToPoseNode::result_callback(const RequestHandleOrder::WrappedResult & result){
  auto request = std::make_shared<uahrk_navigation_msgs::srv::SetPose2d::Request>();
  
  order_result = result.code;
  request->pose = result.result->last_odom;
  request->header.frame_id = "odom";
  request->header.stamp = this->get_clock()->now();
  set_odom_tf_client->async_send_request(request);
}


/*
  SERVER FUNCS:

*/
rclcpp_action::GoalResponse MoveToPoseNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Path::Goal> goal){

    RCLCPP_INFO(this->get_logger(), 
      "Received path goal. Path size : %d",
      goal->pose.poses.size());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse MoveToPoseNode::handle_cancel(
  const std::shared_ptr<GoalHandlePath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveToPoseNode::handle_accepted(const std::shared_ptr<GoalHandlePath> goal_handle)
{
  // Update the actual handle variable an
  // acomplish the goal in control_cycle
  actual_wp = 0;
  actual_handle = goal_handle;
  path_goal = std::make_shared<geometry_msgs::msg::PoseArray>(actual_handle->get_goal()->pose);
}

Pose2d MoveToPoseNode::get_robot_pose(){
  rclcpp::Time now = this->get_clock()->now();
  geometry_msgs::msg::TransformStamped robot_tf = tf_buffer_->lookupTransform(
      "map", "robot", now, 100ms);
  return {robot_tf};
}



