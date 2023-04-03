#include "uahrk_decision_making/RunExternBT_BTNode.hpp"


RunExternBT_BTNode::RunExternBT_BTNode(
    const std::string & xml_tag_name, 
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf){
    config().blackboard->get("node", node_);
}


BT::NodeStatus RunExternBT_BTNode::tick(){
    // Servers not ready
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    if(!tree_loaded){
        std::string pkgpath = ament_index_cpp::get_package_share_directory("uahrk_decision_making");

        factory.registerFromPlugin(pkgpath + "/../../lib/libhello_bt_node.so");
        factory.registerFromPlugin(pkgpath + "/../../lib/libgotopose_bt_node.so");
        factory.registerFromPlugin(pkgpath + "/../../lib/libupdateinitstate_bt_node.so");
        factory.registerFromPlugin(pkgpath + "/../../lib/libwaitstartsignal_bt_node.so");
        factory.registerFromPlugin(pkgpath + "/../../lib/librunexternbt_bt_node.so");
        
        std::string tree_name;
        std::string ally_tree_name;

        getInput("tree", tree_name);
        getInput("ally_tree", ally_tree_name);
        std::string xml_file = pkgpath + "/behavior_trees/" + tree_name + ".xml";
        

        auto blackboard = BT::Blackboard::create();
        blackboard->set("node", node_);
        blackboard->set("ally_tree", ally_tree_name);


        tree_ = factory.createTreeFromFile(xml_file, blackboard);
        tree_loaded = true;
    }   

    auto state =  tree_.rootNode()->executeTick();  
    if(state != BT::NodeStatus::RUNNING){
        tree_loaded = false;
    }
    std::cout << "state " << state << std::endl;
    ////auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1670, 1671);
    return state;
}

void RunExternBT_BTNode::halt(){
    return;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<RunExternBT_BTNode>("RunExternBT");
}

