#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include <memory>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    // Create a ROS NODE
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("DecisionTree");
   
    // Register all of the BT Nodes
    BT::BehaviorTreeFactory factory;
    std::string pkgpath = ament_index_cpp::get_package_share_directory("uahrk_decision_making");
    factory.registerFromPlugin(pkgpath + "/../../lib/libhello_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/libgotopose_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/libwaitstart_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/librunexternbt_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/libresetparams_bt_node.so");

    // Create the tree
    // Get the tree file
    std::string xml_file = pkgpath + "/behavior_trees/system_tree.xml";

    // Create a blackboard and store the ROS Node in a parameter
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

    // Create a publisher to debug the tree with Groot    
    auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

    // Run the tree until ros dies with a rate of 10 Hz
    rclcpp::Rate rate(10);
    while (rclcpp::ok()){
        tree.rootNode()->executeTick();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
