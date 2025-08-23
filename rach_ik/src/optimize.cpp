#include <rach_ik/optimize.h>
#include <rach_ik/constraint.h>
#include <rach_ik/objective.h>
#include <rach_ik/conversion_utils.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <math.h>
#include <stdio.h>

#include <vector>
#include <iostream>
#include <iomanip>
#include <nlopt.hpp>

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

    return true;
}

void RachOptimizer::setUserInterfaces() {

    // Offer a service 
    srv_ik_ = this->create_service<reachability_msgs::srv::GetIKPose>("get_ik_pose", 
                        std::bind(&RachOptimizer::handleIKRequest, this, _1, _2));
}


void RachOptimizer::handleIKRequest(const std::shared_ptr<reachability_msgs::srv::GetIKPose::Request> req,
                                    std::shared_ptr<reachability_msgs::srv::GetIKPose::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Handling IK request");
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

     RCLCPP_INFO(this->get_logger(), "Getting configuration");
    res->success = getConfiguration(group, req->goal_pose, req->init_joint_state, res->solution);
    RCLCPP_INFO(this->get_logger(), "Returned with success value: %d", res->success);

    return;
}

/**
 * @function getConfiguration
 */
bool RachOptimizer::getConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_pose,
                const sensor_msgs::msg::JointState &_js,
                sensor_msgs::msg::JointState &_sol) {

Eigen::Isometry3d Tfx_root_ref, Tfx_ref, Tfx_root;
getTransform(group_info_[_group].root_link, _pose.header.frame_id, Tfx_root_ref);
tf2::fromMsg(_pose.pose, Tfx_ref);
Tfx_root = Tfx_root_ref * Tfx_ref;

nlopt::opt opt(nlopt::LD_MMA, group_info_[_group].joint_names.size());
//nlopt::opt opt(nlopt::LN_COBYLA, group_info_[_group].joint_names.size());

opt.set_lower_bounds(group_info_[_group].lower_bounds);
opt.set_upper_bounds(group_info_[_group].upper_bounds);

// Fill ObjectiveData
ObjectiveData od;
od.fk_solver = group_info_[_group].fk_solver;

od.goal_pos = Tfx_root.translation();
od.goal_rot = Tfx_root.rotation();
RCLCPP_INFO(this->get_logger(), "Pos: %f %f %f. ", od.goal_pos.x(), od.goal_pos.y(), od.goal_pos.z());
opt.set_min_objective(cost_function, &od);

//constraint_data data[2] = { {2, 0}, {-1, 1}};
//opt.add_inequality_constraint(constraint_lin, &data[0], 1e-8);
//opt.add_inequality_constraint(constraint_lin, &data[1], 1e-8);

opt.set_xtol_rel(1e-4);

std::vector<double> x;
// Hack, if not current joint received, use zeros
sensor_msgs::msg::JointState js;
js = _js;
if(js.position.empty())
{
    js.name = group_info_[_group].joint_names;
    js.position = std::vector<double>(js.name.size(), 0.0);
}

jointStateToVector(js, x);

double minf;
bool ret;

try {
    nlopt::result result = opt.optimize(x, minf);
    RCLCPP_INFO(this->get_logger(), "Found minimum");
    _sol.name = js.name;
    _sol.position = x;
    ret = true;
} catch(std::exception &e) {
    RCLCPP_INFO(this->get_logger(), "nlopt failed: %s", e.what() );
    ret = false;
}

return ret;
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

