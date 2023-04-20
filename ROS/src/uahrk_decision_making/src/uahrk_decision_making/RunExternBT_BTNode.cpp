#include "uahrk_decision_making/RunExternBT_BTNode.hpp"

#include <filesystem>
RunExternBT_BTNode::RunExternBT_BTNode(const std::string &xml_tag_name,
                                       const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {
  config().blackboard->get("node", node_);
}

BT::NodeStatus RunExternBT_BTNode::tick() {
  // Servers not ready
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  if (!tree_loaded) {
    std::string pkgpath =
        ament_index_cpp::get_package_share_directory("uahrk_decision_making");

    // factory.registerFromPlugin(pkgpath + "/../../lib/libhello_bt_node.so");
    // factory.registerFromPlugin(pkgpath + "/../../lib/libgotopose_bt_node.so");
    // factory.registerFromPlugin(pkgpath +
    //                            "/../../lib/libupdateinitstate_bt_node.so");
    // factory.registerFromPlugin(pkgpath +
    //                            "/../../lib/libwaitstartsignal_bt_node.so");
    // factory.registerFromPlugin(pkgpath +
    //                            "/../../lib/librunexternbt_bt_node.so");

    // TODO: add all .so files inside that directory you dumb fuck
    std::filesystem::path search_path{pkgpath};
    search_path = search_path / ".." / ".." / "lib";
    for (auto const &dir_entry :
         std::filesystem::directory_iterator{search_path}) {
      if (dir_entry.is_regular_file()) {
        auto extension = dir_entry.path().extension().string();
        if (extension == ".so") {
          std::cout << "Found tree: " << dir_entry.path().filename() << '\n';
          factory.registerFromPlugin(dir_entry.path().filename());
        }
      }
    }

    std::string tree_name;
    std::string ally_tree_name;

    getInput("tree", tree_name);
    getInput("ally_tree", ally_tree_name);
    std::string xml_file = pkgpath + "/behavior_trees/" + tree_name + ".tree.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node_);
    blackboard->set("ally_tree", ally_tree_name);
    // Get current time
    auto current_time = std::chrono::high_resolution_clock::now();
    uint64_t start_time = std::chrono::time_point_cast<std::chrono::milliseconds>(current_time).time_since_epoch().count();
    blackboard->set("game_start_timestamp", start_time);

    std::cout << "Running tree " << xml_file << '\n';
    tree_ = factory.createTreeFromFile(xml_file, blackboard);
    // publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree_, 10, 1670, 1671);

    tree_loaded = true;
  }

  auto state = tree_.rootNode()->executeTick();
  if (state != BT::NodeStatus::RUNNING) {
    tree_loaded = false;
  }
  std::cout << "state " << state << std::endl;
  return state;
}

void RunExternBT_BTNode::halt() { return; }

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<RunExternBT_BTNode>("RunExternBT");
}
