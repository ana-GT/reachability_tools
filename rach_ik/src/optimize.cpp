#include <rach_ik/optimize.h>
#include <rach_ik/objective.h>

#include <math.h>
#include <stdio.h>

#include <vector>
#include <iostream>
#include <iomanip>

using std::placeholders::_1;
using std::placeholders::_2;

RachOptimizer::RachOptimizer() :
rclcpp::Node("rach_optimizer") {
    this->declare_parameter("robot_description", std::string(""));
    this->declare_parameter("robot_description_semantic", std::string(""));
    
    robot_entity_.reset(new RobotEntity());
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/**
 * @function init
 */
bool RachOptimizer::init() {

    this->get_parameter("robot_description", urdf_string_);
    this->get_parameter("robot_description_semantic", srdf_string_);

    if(urdf_string_.empty())
        return false;

    if(srdf_string_.empty())
        return false;

    // Initialize the robot entity
    if(!robot_entity_->init(urdf_string_, srdf_string_))
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't load robot entity");
        return false;
    }    

    setUserInterfaces();

    return init_();
}

void RachOptimizer::setUserInterfaces() {

    // Offer a service 
    srv_ik_ = this->create_service<reachability_msgs::srv::GetIKPose>("get_ik_pose", 
                        std::bind(&RachOptimizer::handleIKRequest, this, _1, _2));
}


void RachOptimizer::handleIKRequest(const std::shared_ptr<reachability_msgs::srv::GetIKPose::Request> req,
                                    std::shared_ptr<reachability_msgs::srv::GetIKPose::Response> res)
{
    // Start assuming no solution is found
    res->success = false;

    // Check if group exists
    auto group = req->group_name;
    if(group_info_.find(group) == group_info_.end())
    {
       reachability_msgs::msg::ChainInfo ci;
       if(!robot_entity_->getChainInfo(group, ci))
        return;

       // Fill it up
       robot_entity_->getKDLChain(ci.root_link, ci.tip_link, group_info_[group].chain);

       group_info_[group].fk_solver.reset(new KDL::ChainFkSolverPos_recursive(group_info_[group].chain));
       
       std::vector<std::pair<double, double>> joint_limits;
    
       if(!robot_entity_->getJointLimits(ci.joint_names, joint_limits))
        return;         


       group_info_[group].root_link = ci.root_link;
       group_info_[group].tip_link = ci.tip_link;
       group_info_[group].joint_names = ci.joint_names;
       for(auto iter : joint_limits)
       {
        group_info_[group].lower_bounds.push_back(iter.first);
        group_info_[group].upper_bounds.push_back(iter.second);
       }

    }

    res->success = getConfiguration(group, req->goal_pose, req->init_joint_state, 
                   req->mobile, res->solution, res->base_pose);
    return;
}


/**
 * @function getTransform
 */
bool RachOptimizer::getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx)
{
   geometry_msgs::msg::TransformStamped tfxs;
   try
   {
      tfxs = tf_buffer_->lookupTransform(_source, _target, rclcpp::Time(0), rclcpp::Duration(1, 0));
   }
   catch (tf2::TransformException& ex)
   {
      RCLCPP_ERROR_STREAM(this->get_logger(), "No transform from " << _source << " to " << _target
                                                       << ".  Error: " << ex.what());
      return false;
   }
   
   _Tfx = tf2::transformToEigen(tfxs);
   return true;
}

