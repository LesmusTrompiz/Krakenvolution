#include "uahrk_decision_making/ResetParamsBTNode.hpp"



ResetParamsBTNode::ResetParamsBTNode(
    const std::string & xml_tag_name, 
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
    start_condition_client = node_->create_client<std_srvs::srv::Empty>("reset_params");
}


BT::NodeStatus ResetParamsBTNode::tick(){
    // Servers not ready
    if(!start_condition_client->service_is_ready())  return BT::NodeStatus::FAILURE;
    
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result =  start_condition_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result,300ms) 
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
}

void ResetParamsBTNode::halt(){
    return;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<ResetParamsBTNode>("ResetParams");
}

