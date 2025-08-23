#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robot_unit/robot_entity.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <reachability_msgs/srv/get_ik_pose.hpp>
#include <rach_ik/objective.h>


struct GroupIKInfo {
    KDL::Chain chain;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    std::vector<std::string> joint_names;
    std::string root_link;
    std::string tip_link;
};


/**
 * @class RachOptimizer
 */
class RachOptimizer : public rclcpp::Node {

public :
    RachOptimizer();
    bool init();
    bool getConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_pose,
                const sensor_msgs::msg::JointState &_js,
                sensor_msgs::msg::JointState &_sol);

protected:
    bool getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx);
    void setUserInterfaces();
    void handleIKRequest(const std::shared_ptr<reachability_msgs::srv::GetIKPose::Request> req,
                        std::shared_ptr<reachability_msgs::srv::GetIKPose::Response> res);

    rclcpp::Service<reachability_msgs::srv::GetIKPose>::SharedPtr srv_ik_;

    std::string urdf_string_;
    std::string srdf_string_;

    std::map<std::string, GroupIKInfo> group_info_;
    std::shared_ptr<RobotEntity> robot_entity_;

   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;    
};