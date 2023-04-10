#include "uahrk_decision_making/WaitStartSignalBTNode.hpp"



WaitStartSignalBTNode::WaitStartSignalBTNode(
    const std::string & xml_tag_name, 
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
    start_condition_client = node_->create_client<std_srvs::srv::Trigger>("start_condition");
}


BT::NodeStatus WaitStartSignalBTNode::tick(){
    // Servers not ready
    //std::cout << "Wait start signal" << std::endl;
    if(!start_condition_client->service_is_ready())  return BT::NodeStatus::FAILURE;
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result =  start_condition_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result,500ms) 
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        return BT::NodeStatus::FAILURE;
    }

    if(result.get()->success){
        return BT::NodeStatus::SUCCESS;
    }
    else return BT::NodeStatus::FAILURE;
}

void WaitStartSignalBTNode::halt(){
    return;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<WaitStartSignalBTNode>("Wait_start_signal");
}

