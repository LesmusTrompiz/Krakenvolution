#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include <memory>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "uahrk_scenario_msgs/msg/scenario.hpp"
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
    factory.registerFromPlugin(pkgpath + "/../../lib/libcaller_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/libpub_points_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/libpoint_to_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/libfind_basket_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/libfind_closest_spawn_bt_node.so");
    factory.registerFromPlugin(pkgpath + "/../../lib/libfind_closest_cake_bt_node.so");


    // Create the tree
    // Get the tree file
    // std::string xml_file = pkgpath + "/behavior_trees/system_tree.xml";
    factory.registerFromPlugin(pkgpath + "/../../lib/libtimer_bt_node.so");
    std::string xml_file = pkgpath + "/behavior_trees/test.xml";

    // Create a blackboard and store the ROS Node in a parameter
    auto blackboard = BT::Blackboard::create();

    auto current_time = std::chrono::high_resolution_clock::now();
    uint64_t start_time = std::chrono::time_point_cast<std::chrono::milliseconds>(current_time).time_since_epoch().count();
    blackboard->set("game_start_timestamp", start_time);

    blackboard->set("node", node);

    auto scenary_subscriber = node->create_subscription<uahrk_scenario_msgs::msg::Scenario>(
    "/scenario", 10, std::function<void(std::shared_ptr<uahrk_scenario_msgs::msg::Scenario>)>{[&](auto msg) -> void {
        blackboard->set("scenary", msg);
    }});

    // TODO: remove testing
    uahrk_scenario_msgs::msg::Scenario scenario;
    geometry_msgs::msg::Pose2D pose;
    pose.x = 14;
    pose.y = 42;
    pose.theta = 1.0;
    scenario.canasta.push_back(pose);
    auto msg = std::shared_ptr<uahrk_scenario_msgs::msg::Scenario>(&scenario);
    blackboard->set("scenary", msg);

    BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);


    // Create a publisher to debug the tree with Groot    
    // auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

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
