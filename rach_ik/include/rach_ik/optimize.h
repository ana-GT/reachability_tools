#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robot_unit/robot_entity.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <reachability_msgs/srv/get_ik_pose.hpp>
#include <rach_ik/objective.h>

#include <rach_ik/conversion_utils.h>


struct GroupIKInfo {
    KDL::Chain chain;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    std::vector<std::string> joint_names;
    std::string root_link;
    std::string tip_link;
    // Optional
    std::pair<std::string, int> elbow_link;
    std::pair<std::string, int> wrist_link;
};


/**
 * @class RachOptimizer
 */
class RachOptimizer : public rclcpp::Node {

public :
    RachOptimizer();
    bool init();

    virtual bool getConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_pose,
                const sensor_msgs::msg::JointState &_init_js,
                sensor_msgs::msg::JointState &_sol_arm_config) = 0;
    
    virtual bool getMobileConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_goal_pose,
                const sensor_msgs::msg::JointState &_init_js,
                const geometry_msgs::msg::PoseStamped &_init_base_pose,
                sensor_msgs::msg::JointState &_sol_arm_config,
                geometry_msgs::msg::PoseStamped &_sol_base_pose) = 0; 

    bool loadKinematics(const std::string &_group);
protected:
    virtual bool init_() = 0;
   
    
    bool getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx);
    void setUserInterfaces();
    void handleIKRequest(const std::shared_ptr<reachability_msgs::srv::GetIKPose::Request> req,
                        std::shared_ptr<reachability_msgs::srv::GetIKPose::Response> res);

    rclcpp::Service<reachability_msgs::srv::GetIKPose>::SharedPtr srv_ik_;


    // Common
    std::string urdf_string_;
    std::string srdf_string_;
    std::shared_ptr<RobotEntity> robot_entity_;
    
   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
   
   std::map<std::string, GroupIKInfo> group_info_;

};
