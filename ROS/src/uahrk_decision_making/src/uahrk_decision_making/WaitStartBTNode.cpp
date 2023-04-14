#include "uahrk_decision_making/WaitStartBTNode.hpp"


WaitStartBTNode::WaitStartBTNode(
    const std::string & xml_tag_name, 
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
    set_pose_client = node_->create_client<uahrk_navigation_msgs::srv::SetPose2d>("set_pose");
    set_playside_param_client = node_->create_client<rcl_interfaces::srv::SetParameters>("/grid_node/set_parameters");
    node_->declare_parameter("play_side", "default");
    node_->declare_parameter("spawn", "default");
    node_->declare_parameter("tree", "default");
    node_->declare_parameter("ally_tree", "default");
    node_->declare_parameter("start", "default");

}


BT::NodeStatus WaitStartBTNode::tick(){
    // Servers not ready
    if(!set_playside_param_client->service_is_ready())  return BT::NodeStatus::FAILURE;
    if(!set_pose_client->service_is_ready())  return BT::NodeStatus::FAILURE;

    // Retrieve values from params
    std::string play_side =
        node_->get_parameter("play_side").get_parameter_value().get<std::string>();
    std::string key_pose = 
        node_->get_parameter("spawn").get_parameter_value().get<std::string>();
    std::string tree = 
        node_->get_parameter("tree").get_parameter_value().get<std::string>();
    std::string ally_tree = 
        node_->get_parameter("ally_tree").get_parameter_value().get<std::string>();
    



    // Params with default value
    if(play_side == "default") return BT::NodeStatus::FAILURE;
    if(key_pose  == "default") return BT::NodeStatus::FAILURE;
    if(tree      == "default") return BT::NodeStatus::FAILURE;

    // Set blackboard signals
    setOutput("tree", tree );
    setOutput("ally_tree", ally_tree );

    // Update Grid Map
    auto play_side_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto param = std::make_shared<rcl_interfaces::msg::Parameter>(); 
    param->name  = "play_side";
    param->value.string_value = play_side;
    param->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    play_side_request->parameters.push_back(*param);
    auto result =  set_playside_param_client->async_send_request(play_side_request);

    if (rclcpp::spin_until_future_complete(node_, result,300ms) != rclcpp::FutureReturnCode::SUCCESS)
    {
        return BT::NodeStatus::FAILURE;
    }

    // Update Pose 
    if(play_side == "blue"){
        auto pose = blue_spawns[key_pose];
        auto request = std::make_shared<uahrk_navigation_msgs::srv::SetPose2d::Request>();
        request->pose.x = pose.x;
        request->pose.y = pose.y;
        request->pose.theta = pose.a;
        request->header.frame_id = "map";
        request->header.stamp = node_->get_clock()->now();

        auto result = set_pose_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    else if(play_side == "green"){
        auto pose = green_spawns[key_pose];
        auto request = std::make_shared<uahrk_navigation_msgs::srv::SetPose2d::Request>();
        request->pose.x = pose.x;
        request->pose.y = pose.y;
        request->pose.theta = pose.a;
        auto result = set_pose_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result, 300ms) != rclcpp::FutureReturnCode::SUCCESS)
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    else return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}

void WaitStartBTNode::halt(){
    return;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<WaitStartBTNode>("WaitStart");
}

