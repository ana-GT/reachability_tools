/**
 * @file reachability_generation_node.cpp
 */ 
#include <reachability_description/reachability_description.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("load_reachability_node");

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

  // Actually generate the description
  RCLCPP_INFO(node->get_logger(), "Loading description of robot %s and group %s ", robot_name.c_str(), chain_group.c_str());   

  if(!rd.loadDescription(chain_group))
  {
    RCLCPP_INFO(node->get_logger(), "Error loading description from file");   
    return 1;
  }

  // View description
  RCLCPP_INFO(node->get_logger(), "View description of robot %s and group %s ", robot_name.c_str(), chain_group.c_str());   

  rd.viewDescription(chain_group);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;    
}
