/**
 * @file reachability_generation_node.cpp
 */ 
#include <reachability_description/reachability_description.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("reachability_generation_node");
  
  RCLCPP_INFO(node->get_logger(), "Create reachability generation node \n");
  reachability_description::ReachabilityDescription rd(node);
  if(!rd.initialize())
    return 1;

  // Generate 
  RCLCPP_INFO(node->get_logger(), "Generate description start \n");
  rd.quickTest();


  std::string chain_group;
  if(!node->has_parameter("group"))
    node->declare_parameter("group", std::string(""));
  node->get_parameter("group", chain_group);


  rd.generateDescription(chain_group);
  RCLCPP_INFO(node->get_logger(), "Generate description end \n");

  RCLCPP_INFO(node->get_logger(), "Spin! ");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;    
}
