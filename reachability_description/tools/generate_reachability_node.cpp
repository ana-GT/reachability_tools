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
  RCLCPP_INFO(node->get_logger(), "Initialize Reachability Description ");   
  reachability_description::ReachabilityDescription rd(node);
  if(!rd.initialize(robot_name))
  {
    RCLCPP_ERROR(node->get_logger(), "Error initializing Reachability Description");
    RCLCPP_ERROR(node->get_logger(), "Robot name: %s, chain group: %s", robot_name.c_str(), chain_group.c_str());
    return 1;
  }

  // Actually generate the description
  RCLCPP_INFO(node->get_logger(), "generate Description process start ");   
  auto ts = std::chrono::system_clock::now();
  bool b = rd.generateDescription(chain_group);
  auto tf = std::chrono::system_clock::now();
  std::chrono::duration<double> dt = (tf - ts);
  
  RCLCPP_INFO(node->get_logger(), "generate Description Time: %f seconds ", dt.count());   

  if(b)
  {
    // Store the description
    rd.storeDescription(chain_group);

    // View
    rd.viewDescription(chain_group);
  }
  
  RCLCPP_INFO(node->get_logger(), "Spin! ");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;    
}
