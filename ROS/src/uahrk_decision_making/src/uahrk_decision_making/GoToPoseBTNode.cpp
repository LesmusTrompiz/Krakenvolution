#include <string>
#include <iostream>
#include "uahrk_decision_making/GoToPoseBTNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <chrono>
using namespace std::chrono_literals;


GoToPoseBTNode::GoToPoseBTNode(
    const std::string & xml_tag_name, 
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
    client_ptr_ = rclcpp_action::create_client<GoToPose>(node_, "path_finding_server");
    request_state = IDLE;
    if (!client_ptr_->wait_for_action_server(1000ms)){
        throw std::logic_error("Could not contact the Serial Bridge Server. Throwing an exception");
    }
}

geometry_msgs::msg::Quaternion zrot2quaternion(float yaw){
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cy;
    q.x = 0;
    q.y = 0;
    q.z = sy;
    return q;
}


void GoToPoseBTNode::sendGoal(float x, float y, float a){
    GoToPose::Goal goal_msg;
    auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation = zrot2quaternion(a);
    send_goal_options.result_callback        = std::bind(&GoToPoseBTNode::result_callback, this, std::placeholders::_1);
    send_goal_options.goal_response_callback = std::bind(&GoToPoseBTNode::goal_response_callback, this, std::placeholders::_1);
    order_result = rclcpp_action::ResultCode::UNKNOWN;




    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    std::cout << "Order sent" << std::endl;
}

BT::NodeStatus GoToPoseBTNode::tick()
{
    switch(request_state)
    {
        case IDLE:
            float x, y, a;
            getInput("x", x);
            getInput("y", y);
            getInput("a", a);
            //geometry_msgs::msg::PoseStamped goal;
            sendGoal(x, y, a);
            request_state = RUNNING;
            return BT::NodeStatus::RUNNING;
            break;
        case RUNNING:
            
            if(order_result == rclcpp_action::ResultCode::SUCCEEDED){
                std::cout << "SUCCEED";
                request_state = IDLE;
                return BT::NodeStatus::SUCCESS;
            }
    
            else if(order_result == rclcpp_action::ResultCode::ABORTED){
                request_state = IDLE;
                std::cout << "Aborted " << std::endl;
                return BT::NodeStatus::FAILURE;
            }
                
            else if( order_result == rclcpp_action::ResultCode::CANCELED){
                request_state = IDLE;
                std::cout << "Canceled" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
            break;
        default:
            break;
    }
    return BT::NodeStatus::FAILURE;
}


void GoToPoseBTNode::halt(){
    return;
}



void GoToPoseBTNode::result_callback(const GoalHandleGoToPose::WrappedResult & result){
    //node_->RCLCPP_ERROR(this->get_logger(), "Holi el resultado ha llegado");
  
    order_result = result.code;
}

void GoToPoseBTNode::goal_response_callback(std::shared_future<GoalHandleGoToPose::SharedPtr> future){
  auto goal_handle = future.get();
  if (!goal_handle) {
    //RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server. Setting Canceled");
    order_result =  rclcpp_action::ResultCode::CANCELED;

  } else {
    // RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    
  }
}



#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<GoToPoseBTNode>("GoToPose");
}
