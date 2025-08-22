#include <rclcpp/rclcpp.hpp>
#include <rach_ik/optimize.h>



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto ro = std::make_shared<RachOptimizer>();

    RCLCPP_INFO(ro->get_logger(), "Initialize node");    
    if(!ro->init())
        return false;


    RCLCPP_INFO(ro->get_logger(), "Get configuration");    
    ro->getConfiguration();

    return 0;
}