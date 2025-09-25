#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_listener.h>

#include <reachability_msgs/srv/get_mobile_poses.hpp>
#include <reachability_description/reachability_description.h>

// IK
#include <pluginlib/class_loader.hpp>
#include <rach_ik/optimize.h>

/**
 * @class SimpleMobilePose
 **/
class SimpleMobilePose : public rclcpp::Node
{
 public:

  SimpleMobilePose();
  bool initialize(); 
  bool setServices();

 protected:
   bool getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx);
   
   void handleSrv(const std::shared_ptr<reachability_msgs::srv::GetMobilePoses::Request> req,
                  std::shared_ptr<reachability_msgs::srv::GetMobilePoses::Response> res);
 
   rclcpp::Service<reachability_msgs::srv::GetMobilePoses>::SharedPtr srv_;


   // Helpers
   bool getSamplesAtZ(const geometry_msgs::msg::Pose &_pose,
          std::vector<reachability_msgs::msg::ReachData> &_psz,
          const double &_threshold);

   bool getSamplesRatioTop( const std::vector<reachability_msgs::msg::ReachData> &_samples, 
          const double &_top_percent,   
          std::vector<reachability_msgs::msg::ReachData> &_best_samples);

   // Read parameters
   std::string chain_group_;
   std::string robot_name_;
   std::string robot_base_frame_;
            
   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
   

   std::shared_ptr<reachability_description::ReachabilityDescription> rd_;
   reachability_msgs::msg::ReachGraph rg_;
   int min_samples_, max_samples_;
   
   // IK
   std::shared_ptr<RachOptimizer> ro_;

   // Debug
   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_base_poses_;
};
