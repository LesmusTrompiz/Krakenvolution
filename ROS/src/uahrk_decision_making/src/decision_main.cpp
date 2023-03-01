#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include <memory>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("DecisionTree");
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    std::string pkgpath = ament_index_cpp::get_package_share_directory("uahrk_decision_making");

    //factory.registerFromPlugin(loader.getOSName("hello_bt_node"));
    factory.registerFromPlugin(pkgpath + "/../../lib/libhello_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/libgotopose_bt_node.so");


    //factory.registerFromPlugin("/home/trompiz/ws/Krakenvolution/ROS/install/uahrk_decision_making/lib/libhello_bt_node.so");
    
    //factory.registerFromPlugin(loader.getOSName("br2_back_bt_node"));
    //factory.registerFromPlugin(loader.getOSName("br2_turn_bt_node"));
    //factory.registerFromPlugin(loader.getOSName("br2_is_obstacle_bt_node"));
    std::string xml_file = pkgpath + "/behavior_trees/square_tree.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
    
    auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);
    
    rclcpp::Rate rate(10);
    bool finish = false;

    while (!finish && rclcpp::ok()){
        finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
