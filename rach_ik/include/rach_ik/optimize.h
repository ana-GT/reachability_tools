#pragma once

#include <rclcpp/rclcpp.hpp>
#include <reachability_msgs/srv/get_ik_pose.hpp>
#include <rach_ik/objective.h>

/**
 * @class RachOptimizer
 */
class RachOptimizer : public rclcpp::Node {

public :
    RachOptimizer();
    bool init();
    bool getConfiguration();

protected:
    void setUserInterface();
    void handleIKRequest(const std::shared_ptr<reachability_msgs::srv::GetIKPose::Request> req,
                        std::shared_ptr<reachability_msgs::srv::GetIKPose::Response> res)

    rclcpp::Service<reachability_msgs::srv::GetIKPose>::SharedPtr srv_ik_;

    std::string urdf_string_;
    std::string srdf_string_;

    

    CostData cd_;

};