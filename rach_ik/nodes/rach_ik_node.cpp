
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <rach_ik/optimize.h>

/////////////////////////////////////
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    std::string plugin_name;
    auto node = std::make_shared<rclcpp::Node>("rach_ik_node");
    node->declare_parameter("plugin_name", "");
    node->get_parameter("plugin_name", plugin_name);
    
    if(plugin_name.empty())
    {
       RCLCPP_ERROR(node->get_logger(), "'plugin_name' parameter has not been set");
       return 0;
    }
    pluginlib::ClassLoader<RachOptimizer> rach_loader("rach_ik", "RachOptimizer");

    std::shared_ptr<RachOptimizer> ro;
    try
    {
      ro = rach_loader.createSharedInstance(plugin_name);
      if (!ro->init())
      	return 1;
    }
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(node->get_logger(),
      		"The plugin failed to load for some reason. Error: %s\n", 
      		ex.what());
    }

    rclcpp::spin(ro);
    return 0;
}
