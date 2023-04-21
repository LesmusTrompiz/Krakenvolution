#include "uahrk_decision_making/FindBasketBTNode.hpp"

#include <string>
#include <iostream>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uahrk_scenario_msgs/msg/scenario.hpp"

FindBasket::FindBasket( const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
}

BT::NodeStatus FindBasket::tick(){
    std::cout << "Finding basket\n";
    uahrk_scenario_msgs::msg::Scenario::SharedPtr scenario;
    config().blackboard->get("scenary", scenario);
    if (scenario) {
      std::cout << "Found scenario!\n";
      for (auto& val : scenario->canasta) {
        std::cout << val.x << ' ' << val.y << ' ' << val.theta << '\n';
      }
    } else {
      std::cout << "Scenario was not found\n";
    }
    return BT::NodeStatus::RUNNING;
}

void FindBasket::halt(){
    return;
}
 // namespace br2_bt_bumpgo
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<FindBasket>("FindBasket");
}
