#pragma once
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "uahrk_navigation_msgs/srv/set_pose2d.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include <unordered_map>
#include <chrono>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

using namespace std::chrono_literals;




class RunExternBT_BTNode : public BT::ActionNodeBase
{
    public:
        explicit RunExternBT_BTNode(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf);
        BT::NodeStatus tick();
        static BT::PortsList providedPorts(){
            return BT::PortsList({BT::InputPort<std::string>("tree"), BT::InputPort<std::string>("ally_tree")});
            
        }
        void halt();

    private:
        rclcpp::Node::SharedPtr node_;
        BT::Tree tree_;
        bool tree_loaded = false;
        std::shared_ptr<BT::PublisherZMQ> publisher_zmq;
};


