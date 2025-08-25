/**
 * @file optimize_low_elbow.cpp
 */
#include <rach_ik_plugins/optimize_low_elbow.h>
#include <rach_ik_plugins/objectives/objective_ee_diff.h>
#include <rach_ik_plugins/constraints/constraint_elbow_wrist.h>

#include <nlopt.hpp>

namespace rach_ik_plugins
{

/**
 * @function LowElbowOptimizer
 * @brief Constructor
 */
LowElbowOptimizer::LowElbowOptimizer() :
RachOptimizer() {

    this->declare_parameter("reference_poses", std::vector<std::string>());    
    this->declare_parameter("wrist_link", std::string("")); 
    this->declare_parameter("elbow_link", std::string(""));     
}

/**
 * @function init
 */
bool LowElbowOptimizer::init_() {

    this->get_parameter("reference_poses", reference_poses_);
    this->get_parameter("wrist_link", wrist_link_);
    this->get_parameter("elbow_link", elbow_link_);

    if(wrist_link_.empty())
      return false;
      
    if(elbow_link_.empty())
      return false;
     
    return true;
}


/**
 * @function getConfiguration
 */
bool LowElbowOptimizer::getConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_pose,
                const sensor_msgs::msg::JointState &_js,
                sensor_msgs::msg::JointState &_sol) {

  // Optional       
  int index = 0;
  for(auto si : group_info_[_group].chain.segments)
  {
    if(si.getName() == elbow_link_)
       group_info_[_group].elbow_link = std::make_pair(elbow_link_, index);
    else if(si.getName() == wrist_link_)
       group_info_[_group].wrist_link = std::make_pair(wrist_link_, index);

    index++;
  }

  Eigen::Isometry3d Tfx_root_ref, Tfx_ref, Tfx_root;
  getTransform(group_info_[_group].root_link, _pose.header.frame_id, Tfx_root_ref);
  tf2::fromMsg(_pose.pose, Tfx_ref);
  Tfx_root = Tfx_root_ref * Tfx_ref;

  nlopt::opt opt(nlopt::LD_MMA, group_info_[_group].joint_names.size()); //LN_COBYLA

  // Limits
  opt.set_lower_bounds(group_info_[_group].lower_bounds);
  opt.set_upper_bounds(group_info_[_group].upper_bounds);

  // Fill ObjectiveData
  ObjectiveData od;
  od.fk_solver = group_info_[_group].fk_solver;
  od.goal_pos = Tfx_root.translation();
  od.goal_rot = Tfx_root.rotation();
  
  opt.set_min_objective(cost_ee_diff_function, &od);

  // Fill constraint data
  ConstraintData cd;
  cd.fk_solver = group_info_[_group].fk_solver; 
  cd.elbow_index = group_info_[_group].elbow_link.second;
  cd.wrist_index = group_info_[_group].wrist_link.second;

  //opt.add_inequality_constraint(constraint_elbow_down, &cd, 1e-8);

  opt.set_xtol_rel(1e-4);

   std::vector<double> x;
  // Hack, if not current joint received, use zeros
  sensor_msgs::msg::JointState js;
  js = _js;
  if(js.position.empty())
    robot_entity_->getChainGroupState(_group, "pose_left_0_high", js);

  jointStateToVector(js, x);

  double minf;
  bool ret;

  try {
    nlopt::result result = opt.optimize(x, minf);
    _sol.name = js.name;
    _sol.position = x;
    ret = (result >= 0);
    RCLCPP_INFO(this->get_logger(), "*** Found minimum: %d", ret);

  } catch(std::exception &e) {
    RCLCPP_INFO(this->get_logger(), "nlopt failed: %s", e.what() );
    ret = false;
  }

  return ret;
}

} // namespace rach_ik_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rach_ik_plugins::LowElbowOptimizer, RachOptimizer)
