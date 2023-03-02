#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"


template<class ActionT, class NodeT = rclcpp::Node>
class BtActionNode : public BT::ActionNodeBase{
    public:
        BtActionNode(
            const std::string & xml_tag_name,
            const std::string & action_name,
            const BT::NodeConfiguration & conf) 
        : BT::ActionNodeBase(xml_tag_name, conf){
            node_ = config().blackboard->get<typename NodeT::SharedPtr>("node");
        }
        // Could do dynamic checks, such as getting updates to values on the blackboard
        virtual void on_tick()
        {
        }
        // Called upon successful completion of the action. A derived class can override this
        // method to put a value on the blackboard, for example.
        virtual BT::NodeStatus on_success()
        {
            return BT::NodeStatus::SUCCESS;
        }
        // Called when a the action is aborted. By default, the node will return FAILURE.
        // The user may override it to return another value, instead.
        virtual BT::NodeStatus on_aborted()
        {
            return BT::NodeStatus::FAILURE;
            }
        // The main override required by a BT action
        BT::NodeStatus tick() override
        {}
        // The other (optional) override required by a BT action. In this case, we
        // make sure to cancel the ROS2 action if it is still running.
        void halt() override
        {}
    protected:
        typename ActionT::Goal goal_;
};
