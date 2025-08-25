/**
 * @file relaxed_ik_optimize.cpp
 */
#include <rach_ik_plugins/optimize_relaxed_ik.h>
#include <rach_ik_plugins/objectives/objective_relaxed_ik.h>

#include <nlopt.hpp>

namespace rach_ik_plugins
{

/**
 * @function RelaxedIKOptimizer
 * @brief Constructor
 */
RelaxedIKOptimizer::RelaxedIKOptimizer() :
RachOptimizer() {

    this->declare_parameter("reference_poses", std::vector<std::string>());    
}

/**
 * @function init
 */
bool RelaxedIKOptimizer::init_() {

    this->get_parameter("reference_poses", reference_poses_);

    return true;
}

/**
 * @function getConfiguration
 */
bool RelaxedIKOptimizer::getConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_pose,
                const sensor_msgs::msg::JointState &_js,
                sensor_msgs::msg::JointState &_sol) {

  Eigen::Isometry3d Tfx_root_ref, Tfx_ref, Tfx_root;
  getTransform(group_info_[_group].root_link, _pose.header.frame_id, Tfx_root_ref);
  tf2::fromMsg(_pose.pose, Tfx_ref);
  Tfx_root = Tfx_root_ref * Tfx_ref;

  nlopt::opt opt(nlopt::LD_MMA, group_info_[_group].joint_names.size()); // LN_COBYLA, LD_MMA

  opt.set_lower_bounds(group_info_[_group].lower_bounds);
  opt.set_upper_bounds(group_info_[_group].upper_bounds);

  // Fill ObjectiveData
  ObjectiveData od;
  od.fk_solver = group_info_[_group].fk_solver;
  od.goal_pos = Tfx_root.translation();
  od.goal_rot = Tfx_root.rotation();
  
  // Set cost function
  opt.set_min_objective(cost_relaxed_ik_function, &od);

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

} // namespace rach_ik_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rach_ik_plugins::RelaxedIKOptimizer, RachOptimizer)
