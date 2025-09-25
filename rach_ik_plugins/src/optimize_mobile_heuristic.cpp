/**
 * @file optimize_low_elbow.cpp
 */
#include <rach_ik_plugins/optimize_mobile_heuristic.h>
#include <rach_ik_plugins/objectives/objective_mobile_ee_diff.h>
#include <rach_ik_plugins/constraints/constraint_elbow_wrist.h>

#include <nlopt.hpp>

namespace rach_ik_plugins
{

/**
 * @function MobileHeuristicOptimizer
 * @brief Constructor
 */
MobileHeuristicOptimizer::MobileHeuristicOptimizer() :
RachOptimizer() {

    //this->declare_parameter("reference_poses", std::vector<std::string>());    
    this->declare_parameter("wrist_link", std::string("")); 
    this->declare_parameter("elbow_link", std::string(""));     
}

/**
 * @function init
 */
bool MobileHeuristicOptimizer::init_() {

    //this->get_parameter("reference_poses", reference_poses_);
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
bool MobileHeuristicOptimizer::getConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_goal_pose,
                const sensor_msgs::msg::JointState &_js,
                sensor_msgs::msg::JointState &_sol) {

     RCLCPP_ERROR(this->get_logger(), "MobileHeuristicOptimizer only supports mobile IK");
     return false;
}

bool MobileHeuristicOptimizer::getMobileConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_goal_pose,
                const sensor_msgs::msg::JointState &_init_js,
                const geometry_msgs::msg::PoseStamped &_init_base_pose,
                sensor_msgs::msg::JointState &_sol_arm_config,
                geometry_msgs::msg::PoseStamped &_sol_base_pose)
{                
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

  // Cost
  // Tf_world_ee = Tf_world_base * Tf_base_root * FK(q)
  // Base_FK(x, y, theta, q) = Tf(x, y, theta) * Tfixed_base_root * FK(q)
  Eigen::Isometry3d Tf_ref_base, Tf_base_root;
  double x0, y0, alpha0;
  
  tf2::fromMsg(_init_base_pose.pose, Tf_ref_base);
  //getCurrentBasePlacement(_pose.header.frame_id, x0, y0, alpha0);
  fromTfPlanar(Tf_ref_base, x0, y0, alpha0);

  getTransform(robot_entity_->getRootLinkName(), group_info_[_group].root_link, Tf_base_root); 

  int num_vars = group_info_[_group].joint_names.size() + 3;
  nlopt::opt opt(nlopt::LD_MMA, num_vars); //LN_COBYLA

  // Limits
  std::vector<double> lb = group_info_[_group].lower_bounds;
  std::vector<double> ub = group_info_[_group].upper_bounds;
  
  // Add x limit
  double dx; double dangle;
  dx = 0.3;
  dangle = 45.0 * 3.1416/180.0;
  lb.push_back(x0 - dx);
  ub.push_back(x0 + dx);

  lb.push_back(y0 - dx);
  ub.push_back(y0 + dx);

  lb.push_back(alpha0 - dangle);
  ub.push_back(alpha0 + dangle);


  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);

  Eigen::Isometry3d Tf_ref_ee;
  tf2::fromMsg(_goal_pose.pose, Tf_ref_ee);

  // Fill ObjectiveData
  MobileObjectiveData od;
  od.fk_solver = group_info_[_group].fk_solver;
  od.goal_pos = Tf_ref_ee.translation();
  od.goal_rot = Tf_ref_ee.rotation();
  od.Tf_base_root = Tf_base_root; 
  opt.set_min_objective(cost_mobile_ee_diff_function, &od);

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
  js = _init_js;
  if(js.position.empty())
    robot_entity_->getChainGroupState(_group, "pose_left_0_high", js);

  jointStateToVector(js, x);
  x.push_back(x0); x.push_back(y0); x.push_back(alpha0);
  
  double minf;
  bool ret;

  try {
    nlopt::result result = opt.optimize(x, minf);
    _sol_arm_config.name = js.name;
    for(int i = 0; i < num_vars - 3; ++i)
      _sol_arm_config.position.push_back(x[i]);
      
    Eigen::Isometry3d Tfg;
    Tfg = getTfPlanar(x[num_vars-3], x[num_vars-2], x[num_vars-1]);  
    _sol_base_pose.pose = tf2::toMsg(Tfg);
    _sol_base_pose.header.frame_id = _goal_pose.header.frame_id;
      
    ret = (result >= 0);
    RCLCPP_INFO(this->get_logger(), "*** Found minimum: %d", ret);

  } catch(std::exception &e) {
    RCLCPP_INFO(this->get_logger(), "nlopt failed: %s", e.what() );
    ret = false;
  }

  return ret;
}

void MobileHeuristicOptimizer::getCurrentBasePlacement(const std::string &_ref_frame, double &_x0, double &_y0, double &_yaw0)
{
  // Get transform base_root
  Eigen::Isometry3d Tfx_ref_base;
  std::string robot_base = robot_entity_->getRootLinkName();
  
  // Get init values for x
  getTransform(_ref_frame, robot_base, Tfx_ref_base); 
  fromTfPlanar(Tfx_ref_base, _x0, _y0, _yaw0);
}

} // namespace rach_ik_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rach_ik_plugins::MobileHeuristicOptimizer, RachOptimizer)
