/**
 * @file estimate_reachability_limits_node.cpp
 */ 
#include <reachability_description/reachability_description.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("estimate_reachability_limits_node");

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
  auto ts = std::chrono::system_clock::now();
  rd.estimateReachLimits(chain_group);
  auto tf = std::chrono::system_clock::now();
  std::chrono::duration<double> dt = (tf - ts);
  RCLCPP_INFO(node->get_logger(), "Estimate reach limits for robot %s and group %s. Time: %f seconds ", robot_name.c_str(), chain_group.c_str(), dt.count());   

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;    
}
