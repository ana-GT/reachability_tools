/**
 * @file reachability_generation_node.cpp
 */ 
#include <reachability_description/reachability_description.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("reachability_generation_node");

  // Read parameters
  std::string chain_group;
  std::string robot_name;
  if(!node->has_parameter("chain_group_name"))
    node->declare_parameter("chain_group_name", std::string(""));
  node->get_parameter("chain_group_name", chain_group);

  if(!node->has_parameter("robot_name"))
    node->declare_parameter("robot_name", std::string(""));
  node->get_parameter("robot_name", robot_name);

  // Create main class and initialize  
  reachability_description::ReachabilityDescription rd(node);
  if(!rd.initialize(robot_name))
    return 1;

  // Quick debug test
  //rd.quickTest(chain_group);

  // Actually generate the description
  if(!rd.generateDescription(chain_group))
    return 1;

  // Store the description
  rd.storeDescription();

  // View
  rd.viewDescription();

  RCLCPP_INFO(node->get_logger(), "Spin! ");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;    
}
