#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("carry_my_luggage_node");
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("dialog_confirmation"));
  factory.registerFromPlugin(loader.getOSName("go_back"));
  factory.registerFromPlugin(loader.getOSName("is_detected"));
  factory.registerFromPlugin(loader.getOSName("is_entity_moving"));
  factory.registerFromPlugin(loader.getOSName("move_arm_to_predefined"));
  factory.registerFromPlugin(loader.getOSName("move_to"));
  factory.registerFromPlugin(loader.getOSName("speak"));
  factory.registerFromPlugin(loader.getOSName("is_pointing"));
  

  std::string pkgpath = ament_index_cpp::get_package_share_directory("robocup_behavior_trees");
  std::string xml_file = pkgpath + "/xml/carry_my_luggage.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

  rclcpp::Rate rate(10);
  
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  bool finish = false;
  while (!finish && rclcpp::ok()) {
    status = tree.rootNode()->executeTick();
    finish = (status == BT::NodeStatus::SUCCESS) || (status == BT::NodeStatus::FAILURE);
    

    rclcpp::spin_some(node);
    rate.sleep();
  }
  std::cout << "Carry my luggage Finished with status: "<< status << std::endl;
  rclcpp::shutdown();
  return 0;
}