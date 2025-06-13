/**
 * @file reachability_description.cpp 
 */
#include <reachability_description/reachability_description.h>
#include <reachability_description/reach_serialization.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <reachability_description/reach_utilities.h>


namespace reachability_description
{

const auto logger = rclcpp::get_logger("reachability_description");

/**
 * @function ReachabilityDescription
 * @brief Constructor 
 */
ReachabilityDescription::ReachabilityDescription(const rclcpp::Node::SharedPtr &_node) :
node_(_node)
{
 reach_graph_loader_.reset( new pluginlib::ClassLoader<reachability_description::ReachGraph>("reachability_description", "reachability_description::ReachGraph"));
}

/**
 * @function ~ReachabilityDescription
 * @brief Destructor
 */
ReachabilityDescription::~ReachabilityDescription()
{

}

/**
 * @function initialize
 * @brief Read robot's group information and creates RobotEntity() and RobotCollisionObject()
 */
bool ReachabilityDescription::initialize(const std::string &_robot_name)
{
  robot_name_ = _robot_name;

  RCLCPP_WARN(logger, "Initialize - getting parameters");
  // Declare parameters
  if(!node_->has_parameter("robot_description"))
    node_->declare_parameter("robot_description", std::string(""));

  if(!node_->has_parameter("robot_description_semantic"))
    node_->declare_parameter("robot_description_semantic", std::string(""));

  if(!node_->has_parameter("plugin_name"))
    node_->declare_parameter("plugin_name", std::string(""));

  // Get information
  node_->get_parameter("robot_description", urdf_string_);
  node_->get_parameter("robot_description_semantic", srdf_string_);
  node_->get_parameter("plugin_name", plugin_name_);
    
  if(plugin_name_.empty())
  {
     RCLCPP_ERROR(logger, "'plugin_name' parameter not set!");
     return false;
  }

  // Initialize RobotEntity (provides kinematics info)
  re_.reset(new RobotEntity());
  if(!re_->init(urdf_string_, srdf_string_))
      return false;

  // Initializes RobotCollisionObject (probably not needed at this high-level, only for reachability, self-coll checks)
  rco_.reset(new robot_unit::RobotCollisionObject());
  if(!rco_->init(DEFAULT_REF_FRAME, robot_name_, urdf_string_, srdf_string_))
    return false;

  // Publishes reachability result
  rclcpp::QoS qos_latch(1);
  qos_latch.transient_local();

  pub_reach_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(REACH_CLOUD_TOPIC, qos_latch);
  pub_reach_graph_ = node_->create_publisher<reachability_msgs::msg::ReachGraphStamped>(REACH_GRAPH_TOPIC, qos_latch);
  return true;
}

/**
 * @function initializeGroup
 */
bool ReachabilityDescription::initializeGroup(const std::string &_chain_group)
{
  // Parameters
  if(params_.find(_chain_group) == params_.end())
    loadParams(_chain_group, params_[_chain_group]);

  if(chain_info_.find(_chain_group) == chain_info_.end())
    addKinematicSolvers(_chain_group);

   RCLCPP_WARN(logger, "Generate reach data for robot %s, group: %s, min: (%f %f %f), max: (%f %f %f), res: %f, # voxel samples: %ld",
   robot_name_.c_str(), _chain_group.c_str(), 
   params_[_chain_group].x_min, params_[_chain_group].y_min, params_[_chain_group].z_min, 
   params_[_chain_group].x_max, params_[_chain_group].y_max, params_[_chain_group].z_max, 
   params_[_chain_group].voxel_resolution, params_[_chain_group].voxel_num_samples);

  // Init reach graph
  reachability_msgs::msg::ReachData reach_default;
  reach_default.state = reachability_msgs::msg::ReachData::NO_FILLED;

  if(!initializeReachGraph(reach_graph_[_chain_group], chain_info_[_chain_group], 
                 params_[_chain_group].x_min, params_[_chain_group].y_min, params_[_chain_group].z_min, 
                 params_[_chain_group].x_max, params_[_chain_group].y_max, params_[_chain_group].z_max,
	         params_[_chain_group].voxel_resolution, params_[_chain_group].voxel_num_samples, reach_default ))
     return false;	         

  return true;
}

/**
 * @function calculateReachabilityPoint
 */
reachability_msgs::msg::ReachData ReachabilityDescription::calculateReachabilityPoint(double _x, double _y, double _z,
                                          const std::string &_chain_group)
{
  KDL::JntArray q_init;
  q_init.data = Eigen::VectorXd::Zero( chain_info_[_chain_group].joint_names.size() );
  return calculateReachabilityPoint(reach_graph_[_chain_group], _x, _y, _z,
         ik_solver_[_chain_group], chain_info_[_chain_group], rco_, q_init);
}

/**
 * @function generateDescription 
 */
bool ReachabilityDescription::generateDescription(const std::string &_chain_group)
{
  double x_min, y_min, z_min;
  double x_max, y_max, z_max;
  double x_mid, y_mid, z_mid;

  x_min = reach_graph_[_chain_group]->getMinX();
  x_max = reach_graph_[_chain_group]->getMaxX();  
  x_mid = ( x_min + x_max )*0.5;
  
  y_min = reach_graph_[_chain_group]->getMinY();
  y_max = reach_graph_[_chain_group]->getMaxY();
  y_mid = ( y_min + y_max )*0.5;
  
  z_min = reach_graph_[_chain_group]->getMinZ();
  z_max = reach_graph_[_chain_group]->getMaxZ();
  z_mid = ( z_min + z_max )*0.5;
  
  std::map<std::string, KDL::JntArray > joint_configs;
  for(auto jc :  params_[_chain_group].ik_start_configs)
    re_->getChainGroupState(_chain_group, jc, joint_configs[jc]);

  // Get the FK for these joint configs
  std::map<std::string, KDL::Frame> fk_poses;
  for(auto ji : joint_configs)
    fk_solver_[_chain_group]->JntToCart(ji.second, fk_poses[ji.first]);
  
  

 std::thread to1_( &ReachabilityDescription::reach_calc, this, 
                  x_min, y_min, z_min, 
                  x_mid, y_mid, z_max, 
                  chain_info_[_chain_group], 
                  params_[_chain_group].ik_max_time, params_[_chain_group].ik_epsilon, stringToType(params_[_chain_group].ik_type), 
                  joint_configs, fk_poses);
 std::thread to2_( &ReachabilityDescription::reach_calc, this, 
                  x_min, y_mid, z_min, 
                  x_mid, y_max, z_max, 
                  chain_info_[_chain_group], 
                  params_[_chain_group].ik_max_time, params_[_chain_group].ik_epsilon, stringToType(params_[_chain_group].ik_type), 
                  joint_configs, fk_poses);
 std::thread to3_( &ReachabilityDescription::reach_calc, this, 
                  x_mid, y_min, z_min, 
                  x_max, y_mid, z_max, 
                  chain_info_[_chain_group], 
                  params_[_chain_group].ik_max_time, params_[_chain_group].ik_epsilon, stringToType(params_[_chain_group].ik_type), 
                  joint_configs, fk_poses);
 std::thread to4_( &ReachabilityDescription::reach_calc, this, 
                  x_mid, y_mid, z_min, 
                  x_max, y_max, z_max, 
                  chain_info_[_chain_group], 
                  params_[_chain_group].ik_max_time, params_[_chain_group].ik_epsilon, stringToType(params_[_chain_group].ik_type), 
                  joint_configs, fk_poses);

 to1_.join();
 to2_.join();
 to3_.join();
 to4_.join();

 return true;
}

/**
 * @function viewDescription 
 */
bool ReachabilityDescription::viewDescription(const std::string &_chain_group)
{
  if(reach_graph_.find(_chain_group) == reach_graph_.end())
    return false;

  std::string plane = params_[_chain_group].display_reachability.plane;
  double plane_dist = params_[_chain_group].display_reachability.plane_dist;

  sensor_msgs::msg::PointCloud2 msg;
  msg = reach_graph_[_chain_group]->getPCD(plane, plane_dist);
  //msg = reach_graph_->getPCDHigherThan(0.2);
/*
  rclcpp::Rate r(1.0);
  for(unsigned int i = 0; i < 10; ++i)
  {
    pub_reach_->publish(msg);
    r.sleep();
    rclcpp::spin_some(node_);
  }
*/
  reachability_msgs::msg::ReachGraphStamped rgs_msg;

  auto rgs = this->getReachGraph(_chain_group);  
  rgs_msg.data.chain_info = rgs->getChainInfo();
  //rgs_msg.data.params = ;
  rgs_msg.header.stamp = node_->now();
  rgs_msg.header.frame_id = rgs_msg.data.chain_info.root_link;

  for(int i = 0; i < rgs->getNumPoints(); ++i)
  { 
     rgs_msg.data.points.push_back(rgs->getState(i));
  }
  RCLCPP_INFO(logger, "Number of points %d being sent!!!!!", rgs_msg.data.points.size());  
  pub_reach_graph_->publish(rgs_msg);

  return true;
}

/**
 * @function storeDescription
 */
bool ReachabilityDescription::loadDescription(const std::string &_chain_group)
{
  /*if(reach_graph_.find(_chain_group) != reach_graph_.end())
  {
    RCLCPP_INFO(logger, "Reach description for %s will not be loaded  because it already exists", 
                _chain_group.c_str());
    return false;
  }*/

  std::string filename = generateDefaultReachGroupName(_chain_group);
  RCLCPP_INFO(logger, "Trying to load from file: %s", filename.c_str());
  reachability_msgs::msg::ReachGraph msg;
  
  ReachSerial<reachability_msgs::msg::ReachGraph> reach_serial;

  bool b = reach_serial.readFromDisk(filename, msg);

  if(!b)
    return false;

  // Load ReachData with message
  return initializeReachGraph(reach_graph_[_chain_group], msg);
}


/**
 * @function storeDescription
 */
bool ReachabilityDescription::storeDescription(const std::string &_chain_group)
{
  if(reach_graph_.find(_chain_group) == reach_graph_.end())
  {
    RCLCPP_INFO(logger, "Cannot store reach description for %s . It is not generated / loaded ", 
                _chain_group.c_str());
    return false;
  }

  std::string filename = generateDefaultReachGroupName(_chain_group);
  
  RCLCPP_WARN(logger, "Storing description for robot %s and group %s in %s", 
              robot_name_.c_str(), _chain_group.c_str(), filename.c_str());

  reachability_msgs::msg::ReachGraph msg;
  reach_graph_[_chain_group]->toMsg(msg);
  
  ReachSerial<reachability_msgs::msg::ReachGraph> reach_serial;
  return reach_serial.writeToDisk(msg, filename);
}



//////////////////////////////////////////
// GETTERS
//////////////////////////////////////////

/**
 * @function getReachabilityData 
 */
bool ReachabilityDescription::getReachabilityData(const std::string &_chain_group,
                                                  const double &_x, const double &_y, const double &_z, 
                                                  reachability_msgs::msg::ReachData &_data)
{
  if(reach_graph_.find(_chain_group) == reach_graph_.end() )
    return false;

  int xi, yi, zi;
  if(!reach_graph_[_chain_group]->worldToVertex(_x, _y, _z, xi, yi, zi))
    return false;

  reachability_msgs::msg::ReachData rdata = reach_graph_[_chain_group]->getState(xi, yi, zi);
  RCLCPP_WARN(logger, "Got reachability data: from %f %f %f to %d %d %d, samples: %ld", 
              _x, _y, _z, xi, yi, zi, rdata.samples.size());

  _data = rdata;
  return true;
}

/**
 * @function getChainInfo 
 */
bool ReachabilityDescription::getChainInfo(const std::string &_chain_group,
                                           reachability_msgs::msg::ChainInfo &_chain_info)
{
  if(chain_info_.find(_chain_group) == chain_info_.end())
    return false;

  _chain_info = chain_info_[_chain_group];
  return true;
}


/**
 * @function getReachGraph 
 */
std::shared_ptr<ReachGraph> ReachabilityDescription::getReachGraph(const std::string &_chain_group)
{
  if(reach_graph_.find(_chain_group) == reach_graph_.end())
  {
    RCLCPP_ERROR(logger, "Cannot get reach graph for %s as it is not loaded", _chain_group.c_str());
    return nullptr;
  }

  return reach_graph_[_chain_group];
}

/**
 * @function getIKSolver 
 */
std::shared_ptr<TRAC_IK::TRAC_IK> ReachabilityDescription::getIKSolver(const std::string &_chain_group)
{
  if(ik_solver_.find(_chain_group) == ik_solver_.end())
  {
    RCLCPP_ERROR(logger, "Cannot get iksolver for %s as it is not loaded", _chain_group.c_str());
    return nullptr;
  }

  return ik_solver_[_chain_group];
}

/**
 * @function getJointLimits 
 */
std::vector<std::pair<double, double>> ReachabilityDescription::getJointLimits(const std::string &_chain_group)
{ 
  std::vector<std::pair<double, double>> res;

  if(joint_limits_.find(_chain_group) == joint_limits_.end())
  {
    RCLCPP_ERROR(logger, "Cannot get joint limits for chain group %s as it is not loaded", _chain_group.c_str());
    return res;
  }

  return joint_limits_[_chain_group];  
}



//////////////////////////////////////
// PROTECTED
//////////////////////////////////////


// "reachability_description::ReachGraphReuleaux"
/**
 * @function createPluginInstance
 * @brief Load plugin for ReachGraph (which calculates sampling in different ways)
 */
bool ReachabilityDescription::createPluginInstance(std::shared_ptr<ReachGraph> &_rg, 
                                                   const std::string &_plugin_name)
{
  try
  {
    _rg = reach_graph_loader_->createSharedInstance(_plugin_name);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(logger, "The plugin failed to load for some reason. Error: %s\n", ex.what());
    return false;
  }
  
  return true;
}

/**
 * @function initializeReachGraph
 * @brief Create plugin for reachGraph and initialize it
 */
bool ReachabilityDescription::initializeReachGraph( std::shared_ptr<ReachGraph> &_rg, 
                    const reachability_msgs::msg::ChainInfo &_ci, 
                    double _min_x, double _min_y, double _min_z,
	            double _max_x, double _max_y, double _max_z,
	            const double &_resolution, 
                    const uint16_t &_voxel_samples,
	            const reachability_msgs::msg::ReachData &_reach_default)
{
  if(!createPluginInstance(_rg, plugin_name_))
    return false;
      
  return _rg->initialize(_ci, _min_x, _min_y, _min_z, _max_x, _max_y, _max_z, _resolution, _voxel_samples, _reach_default);
}

/**
 * @function initializeReachGraph
 * @brief Create plugin for reachGraph and initialize it
 */
bool ReachabilityDescription::initializeReachGraph( std::shared_ptr<ReachGraph> &_rg, const reachability_msgs::msg::ReachGraph &_msg)
{
  if(!createPluginInstance(_rg, plugin_name_))
    return false;

  return  _rg->initialize(_msg);  
}


/**
 * @brief Calculate a section of the reachability graph  
 * IMPORTANT: Create its own IK solver and CollisionObject. DO NOT use the class's members. This
 * function is ran in a thread, so you need unique ik and co objects
 */
void ReachabilityDescription::reach_calc( const double &_min_x, const double &_min_y, const double &_min_z,
                                          const double &_max_x, const double &_max_y, const double &_max_z,
                                          const reachability_msgs::msg::ChainInfo &_ci,
                                          const double &_ik_max_time, const double &_ik_epsilon, 
                                          const TRAC_IK::SolveType &_ik_type,
                                          const std::map<std::string, KDL::JntArray > &_joint_configs,
                                          const std::map<std::string, KDL::Frame> &_fk_poses)
{
  int found_sols = 0;

  std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
  ik_solver.reset( new TRAC_IK::TRAC_IK(node_, _ci.root_link, _ci.tip_link, 
                   "robot_description", 
                   _ik_max_time, _ik_epsilon, _ik_type));

  reachability_msgs::msg::ReachData reach_default;
  reach_default.state = reachability_msgs::msg::ReachData::NO_FILLED;

  std::shared_ptr<ReachGraph> reach_graph_i;
  if(!initializeReachGraph(reach_graph_i, _ci, _min_x, _min_y, _min_z, _max_x, _max_y, _max_z,
       reach_graph_[_ci.group]->getResolution(), 
       reach_graph_[_ci.group]->getNumVoxelSamples(),
       reach_default))
    return;

  std::shared_ptr<robot_unit::RobotCollisionObject> rco;
  rco.reset( new robot_unit::RobotCollisionObject());
  if(!rco->init(DEFAULT_REF_FRAME, robot_name_, urdf_string_, srdf_string_))
    return;

  RCLCPP_WARN(logger, "reach_calc: Starting loop to generate reachability checks for [%.3f %.3f %.3f --- %.3f %.3f %.3f]", _min_x, _min_y, _min_z, _max_x, _max_y, _max_z);
  for(int xi = 0; xi < reach_graph_i->getNumX(); ++xi )
  {
    for(int yi = 0; yi < reach_graph_i->getNumY(); ++yi)
    {
      for(int zi = 0; zi < reach_graph_i->getNumZ(); ++zi)
      {
        KDL::JntArray q_init;
        q_init = getClosestJointConfig(xi, yi, zi, _joint_configs, _fk_poses);
        reachability_msgs::msg::ReachData rdata = calculateReachabilityPoint(reach_graph_i, 
                                                  xi, yi, zi, 
                                                  ik_solver, _ci, rco, 
                                                  q_init);
        reach_graph_i->setState(xi, yi, zi, rdata);

        if(rdata.state == reachability_msgs::msg::ReachData::FILLED)
          found_sols++;

      } // for zi
    } // for yi
 } // for xi 

//--  Fill reach start
reach_fill_mutex_.lock();
for(int i = 0; i < reach_graph_i->getNumPoints(); i++)
{
  int xli, yli, zli, xi, yi, zi;
  double x, y, z;

  reach_graph_i->indexToVertex(i, xli, yli, zli);
  reachability_msgs::msg::ReachData data = reach_graph_i->getState(i);

  // Transform index to double then back to index of main reach
  reach_graph_i->vertexToWorld(xli, yli, zli, x, y, z);

  reach_graph_[_ci.group]->worldToVertex(x, y, z, xi, yi, zi);
  reach_graph_[_ci.group]->setState(xi, yi, zi, data);
}
reach_fill_mutex_.unlock();
//-- Fill reach end

RCLCPP_WARN(logger, "Done in thread! Sols: %u/%u . xyz min: (%.3f, %.3f, %.3f) xyz max: (%.3f, %.3f, %.3f)", 
            found_sols, reach_graph_i->getNumPoints(), 
            _min_x, _min_y, _min_z, _max_x, _max_y, _max_z);
}

/**
 * @function getClosestJointConfig 
 */
KDL::JntArray  ReachabilityDescription::getClosestJointConfig(const double &_xi, const double &_yi, const double &_zi, 
                                     const std::map<std::string, KDL::JntArray> &_joint_configs, 
                                     const std::map<std::string, KDL::Frame> &_fk_poses)
{
  double min_dist = 1000;
  std::string min_state = "";
  double dx, dy, dz, d;

  for(auto fi : _fk_poses)
  {
    dx = fi.second.p.x() - _xi;
    dy = fi.second.p.y() - _yi;
    dz = fi.second.p.z() - _zi;

    d = sqrt(dx*dx + dy*dy + dz*dz);
    if( d < min_dist)
    {
      min_dist = d;
      min_state = fi.first; 
    }
  }

  if(!min_state.empty())
    return _joint_configs.at(min_state);
  else
    return KDL::JntArray();
}


/**
 * @function calculateReachabilityPoint
 */
reachability_msgs::msg::ReachData ReachabilityDescription::calculateReachabilityPoint(const std::shared_ptr<ReachGraph> &_reach_graph, 
                                          int _xi, int _yi, int _zi,
                                          const std::shared_ptr<TRAC_IK::TRAC_IK> &_ik_solver,
                                          const reachability_msgs::msg::ChainInfo &_ci,
                                          const std::shared_ptr<robot_unit::RobotCollisionObject> &_rco,
                                          const KDL::JntArray &_q_init)
{

  double x, y, z;
  _reach_graph->vertexToWorld(_xi, _yi, _zi, x, y, z);

  return calculateReachabilityPoint(_reach_graph, x, y, z, _ik_solver, _ci, _rco, _q_init);
}

/**
 * @function calculateReachabilityPoint
 */
reachability_msgs::msg::ReachData ReachabilityDescription::calculateReachabilityPoint(const std::shared_ptr<ReachGraph> &_reach_graph, 
                                          double _x, double _y, double _z,
                                          const std::shared_ptr<TRAC_IK::TRAC_IK> &_ik_solver,
                                          const reachability_msgs::msg::ChainInfo &_ci,
                                          const std::shared_ptr<robot_unit::RobotCollisionObject> &_rco,
                                          const KDL::JntArray &_q_init)
{
  KDL::JntArray q_out; 
  KDL::Frame p_in; 
  KDL::Twist bounds = KDL::Twist::Zero();

  // Create samples
  reachability_msgs::msg::ReachData rdata;

  std::vector<Eigen::Isometry3d> frames; 
  _reach_graph->generateSamples(_x, _y, _z, frames);

  for(auto frame_i : frames)
  {
    tf2::transformEigenToKDL(frame_i, p_in);

    if(_ik_solver->CartToJnt(_q_init, p_in, q_out, bounds) > 0)
    {
      // Check for self-collisions
      std::vector<KDL::JntArray> sols;
      _ik_solver->getSolutions(sols);

      std::vector<KDL::JntArray> sols_coll_free;
      for(auto qo : sols)
      {
         _rco->update(jntArrayToMsg(qo, _ci));
         if(!_rco->selfCollide() )
            sols_coll_free.push_back(qo);
      }

      if(sols_coll_free.size() > 0)
      {
        reachability_msgs::msg::ReachSample sample_i;
        sample_i.pose = tf2::toMsg(frame_i);
        sample_i.best_config = jntArrayToVector(sols_coll_free.front());
        sample_i.num_configs = sols_coll_free.size();

        rdata.samples.push_back(sample_i);

        } // end selfCollide
    }  // end if  ik_solver

  } // end frames_i

  // Set status
  rdata.state = rdata.samples.size() > 0 ? reachability_msgs::msg::ReachData::FILLED : reachability_msgs::msg::ReachData::NO_FILLED;
  
  // Add metric
  _reach_graph->calculateMetric(rdata);
  return rdata;
}


/**
 * @function isSelfColliding 
 */
bool ReachabilityDescription::isSelfColliding(const sensor_msgs::msg::JointState &_js)
{
  rco_->update(_js);
  return rco_->selfCollide();
}


/**
 * @function addKinematicSolvers
 * @brief Add IK and FK solvers, plus joint limits
 */
bool ReachabilityDescription::addKinematicSolvers(const std::string &_chain_group)
{
  if(params_.find(_chain_group) == params_.end())
    return false;

  reachability_msgs::msg::ChainInfo chain_info;
  re_->getChainInfo(_chain_group, chain_info);

  // Fill chain info
  chain_info_[_chain_group] = chain_info;

  std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver;

  ik_solver.reset( new TRAC_IK::TRAC_IK(node_, chain_info.root_link, 
                              chain_info.tip_link, 
                              "robot_description", 
                              params_[_chain_group].ik_max_time, params_[_chain_group].ik_epsilon, stringToType(params_[_chain_group].ik_type)));

  //  Fill IK solver
  ik_solver_[_chain_group] = ik_solver;

  // Create FIK solver
  KDL::Chain chain;
  getIKSolver(_chain_group)->getKDLChain(chain);
  fk_solver_[_chain_group].reset(new KDL::ChainFkSolverPos_recursive(chain));
   

  // Joint limits
  re_->getJointLimits(chain_info.joint_names, 
                      joint_limits_[_chain_group]);

  return true;
}

/**
 * @function estimateReachLimits
 * @brief Calculate reach limits 
 */
bool ReachabilityDescription::estimateReachLimits(const std::string &_chain_group)
{
  if(ik_solver_.find(_chain_group) == ik_solver_.end())
  {
    RCLCPP_ERROR(logger, "Estimating reachability limits: Group %s was not initialized", _chain_group.c_str());
    return false;
  }
  
  if(params_.find(_chain_group) == params_.end())
  {
    RCLCPP_ERROR(logger, "Estimating reachability limits: Params for %s was not initialized", _chain_group.c_str());
    return false;
  } 

  double max_radius = params_[_chain_group].estimate_reachability.max_radius;
  double res = params_[_chain_group].estimate_reachability.resolution;

  double xmin, ymin, zmin, xmax, ymax, zmax;
  
  double def_min = -1000; 
  double def_max = 1000;
  
  xmin = def_max; ymin = def_max; zmin = def_max;
  xmax = def_min; ymax = def_min; zmax = def_min;

  KDL::JntArray q_init, q_out; 
  KDL::Twist bounds;
  bounds.rot = KDL::Vector(2*M_PI, 2*M_PI, 2*M_PI);
  bounds.vel = KDL::Vector(0.5*res, 0.5*res, 0.5*res);
  q_init.data = Eigen::VectorXd::Zero(chain_info_[_chain_group].num_joints);

  KDL::Frame p_in; 
  p_in = KDL::Frame::Identity();
  
  double x, y, z;
  int n = floor(2*max_radius / res);

  for(int xi = 0; xi < n; ++xi)
  {
    RCLCPP_WARN(logger, "Estimating reachability boundaries %d/%d", xi, n);
    for(int yi = 0; yi < n; ++yi)
    {
      for(int zi = 0; zi < n; ++zi)
      { 
        x = -max_radius + xi*res;
        y = -max_radius + yi*res;
        z = -max_radius + zi*res;

        p_in.p = KDL::Vector(x, y, z);
        int res = ik_solver_[_chain_group]->CartToJnt(q_init, p_in, q_out, bounds);
        if(res > 0)
        {
          if(x < xmin) xmin = x;
          if(y < ymin) ymin = y;
          if(z < zmin) zmin = z;
          if(x > xmax) xmax = x;
          if(y > ymax) ymax = y;
          if(z > zmax) zmax = z;
        }  
      } // for zi
    } // for yi
  } // for xi

  RCLCPP_WARN(logger, "Limits: %f %f %f -- %f %f %f", xmin, ymin, zmin, xmax, ymax, zmax);
  return true;
}

/**
 * @function loadParams 
 */
void ReachabilityDescription::loadParams(const std::string &_chain_group, 
                                         reachability_description_params::Params &_params)
{
  std::string reachability_param_prefix = "reachability_params." + _chain_group;

  std::shared_ptr<reachability_description_params::ParamListener> param_listener;
  param_listener = std::make_shared<reachability_description_params::ParamListener>(node_, reachability_param_prefix);
  _params = param_listener->get_params();
}



/**
 * @function generateDefaultReachGroupName 
 */
std::string ReachabilityDescription::generateDefaultReachGroupName(const std::string &_chain_group)
{
  std::string share_dir;
  share_dir = ament_index_cpp::get_package_share_directory("reachability_description");
  return share_dir + std::string("/reachability_") + robot_name_ + std::string("_") + _chain_group;
}



}  // namespace reachability_description
