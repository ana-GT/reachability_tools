/**
 * @file reachability_description.cpp 
 */
#include <reachability_description/reachability_description.h>
#include <reachability_description/reach_serialization.h>

#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <kdl/chainjnttojacsolver.hpp>


namespace reachability_description
{

/**
 * @function ReachabilityDescription
 * @brief Constructor 
 */
ReachabilityDescription::ReachabilityDescription(const rclcpp::Node::SharedPtr &_nh) :
nh_(_nh)
{

}

/**
 * @function ReachabilityDescription
 * @brief Destructor
 */
ReachabilityDescription::~ReachabilityDescription()
{

}

/**
 * @function initialize
 * @brief Well, initialize
 */
bool ReachabilityDescription::initialize(const std::string &_robot_name,
                                         const double &_max_time, 
                                         const double &_eps, 
                                         const TRAC_IK::SolveType &_ik_type)
{
  robot_name_ = _robot_name;

  // Declare parameters
  if(!nh_->has_parameter("robot_description"))
    nh_->declare_parameter("robot_description", std::string(""));

  if(!nh_->has_parameter("robot_description_semantic"))
    nh_->declare_parameter("robot_description_semantic", std::string(""));

  // Get information
  nh_->get_parameter("robot_description", urdf_string_);
  nh_->get_parameter("robot_description_semantic", srdf_string_);

  // Initialize RobotEntity (provides kinematics info)
  re_.reset(new RobotEntity());
  if(!re_->init(urdf_string_, srdf_string_))
      return false;

  // Initializes RobotCollisionObject (probably not needed at this high-level, only for reachability, self-coll checks)
  rco_.reset(new robot_unit::RobotCollisionObject());
  if(!rco_->init(DEFAULT_REF_FRAME, robot_name_, urdf_string_, srdf_string_))
    return false;

  // Publishes reachability result
  pub_reach_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(REACH_CLOUD_TOPIC, 10);

  return true;
}

/**
 * @brief Create a KDL frame 
 */
KDL::Frame makeKDLFrame(const double &_x, const double &_y, const double &_z, 
                        const double &_roll, const double &_pitch, const double &_yaw)
{
    KDL::Frame pi;
    pi.p = KDL::Vector(_x, _y, _z);
    pi.M = KDL::Rotation::EulerZYX(_yaw, _pitch, _roll);
    return pi;
}

/**
 * @function jntArrayToVector 
 */
std::vector<double> jntArrayToVector(KDL::JntArray _js)
{
  std::vector<double> vec(_js.data.size());
  for(unsigned int i = 0; i < _js.data.size(); ++i)
    vec[i] = _js(i);

  return vec; 
}

/**
 * @function quickTest
 * @brief Specific to panda 
 */
bool ReachabilityDescription::quickTest(const std::string &_chain_group)
{
    RCLCPP_WARN(nh_->get_logger(), "QUICK TEST" );

    // Set rco_ to all zeros and check self-collision
    sensor_msgs::msg::JointState js;
    reachability_msgs::msg::ChainInfo chain_info;
    re_->getChainInfo(_chain_group, chain_info);
    js.name = chain_info.joint_names;


    std::vector<double> jv(chain_info.joint_names.size(), 0);    
    js.position = jv;

    rco_->update(js);
    bool b = rco_->selfCollide();
    RCLCPP_WARN(nh_->get_logger(), "Self collide for all zeros: %d !!!!!", b );

    js.position[3] = -1.57;
    js.position[5] =1.57;

    rco_->update(js);
    b = rco_->selfCollide();
    RCLCPP_WARN(nh_->get_logger(), "Self collide for good pose: %d !!!!!", b );


    return true;
}

/**
 * @brief Calculate a section of the reachability graph  
 */
void ReachabilityDescription::reach_calc( const double &_min_x, const double &_min_y, const double &_min_z,
                                          const double &_max_x, const double &_max_y, const double &_max_z,
                                          const reachability_msgs::msg::ChainInfo &_ci)
{
  int found_sols = 0;

  double max_time = 0.001;
  double eps = 1e-5;
  TRAC_IK::SolveType ik_type = TRAC_IK::SolveType::Distance;

  std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
  ik_solver.reset( new TRAC_IK::TRAC_IK(nh_, _ci.root_link, _ci.tip_link, 
                              "robot_description", 
                              max_time, eps, ik_type));

  reachability_msgs::msg::ReachData reach_default;
  reach_default.state = reachability_msgs::msg::ReachData::NO_FILLED;

  std::shared_ptr<ReachGraph> reach_graph_i;
  reach_graph_i.reset(new ReachGraph( _ci, _min_x, _min_y, _min_z, _max_x, _max_y, _max_z,
  	                                             reach_graph_->getResolution(), 
                                                 reach_graph_->getNumVoxelSamples(),
                                                 reach_default )); 

  std::shared_ptr<robot_unit::RobotCollisionObject> rco;
  rco.reset( new robot_unit::RobotCollisionObject());
  if(!rco->init(DEFAULT_REF_FRAME, robot_name_, urdf_string_, srdf_string_))
    return;


  for(int xi = 0; xi < reach_graph_i->getNumX(); ++xi )
  {
    for(int yi = 0; yi < reach_graph_i->getNumY(); ++yi)
    {
      for(int zi = 0; zi < reach_graph_i->getNumZ(); ++zi)
      {
        reachability_msgs::msg::ReachData rdata = fillData(reach_graph_i, xi, yi, zi, ik_solver, _ci, rco);
        // Store
        reach_graph_i->setState(xi, yi, zi, rdata);

        if(rdata.state == reachability_msgs::msg::ReachData::FILLED)
          found_sols++;

      } // for zi
    } // for yi
 } // for xi 

// Fill reach
reach_fill_mutex_.lock();
for(int i = 0; i < reach_graph_i->getNumPoints(); i++)
{
  int xli, yli, zli, xi, yi, zi;
  double x, y, z;

  reach_graph_i->indexToVertex(i, xli, yli, zli);
  reachability_msgs::msg::ReachData data = reach_graph_i->getState(i);

  // Transform index to double then back to index of main reach
  reach_graph_i->vertexToWorld(xli, yli, zli, x, y, z);
  reach_graph_->worldToVertex(x, y, z, xi, yi, zi);

  reach_graph_->setState(xi, yi, zi, data);
}
reach_fill_mutex_.unlock();

RCLCPP_WARN(rclcpp::get_logger("reach"), "Done in thread! Sols: %u/%u . xyz min: (%.3f, %.3f, %.3f) xyz max: (%.3f, %.3f, %.3f)", 
            found_sols, reach_graph_i->getNumPoints(), 
            _min_x, _min_y, _min_z, _max_x, _max_y, _max_z);
}

/**
 * 
 */
reachability_msgs::msg::ReachData ReachabilityDescription::fillData(const std::shared_ptr<ReachGraph> &_reach_graph, 
                                          int _xi, int _yi, int _zi,
                                          const std::shared_ptr<TRAC_IK::TRAC_IK> &_ik_solver,
                                          const reachability_msgs::msg::ChainInfo &_ci,
                                          const std::shared_ptr<robot_unit::RobotCollisionObject> &_rco)
{
  double x, y, z;
  KDL::JntArray q_init, q_out; 
  KDL::Frame p_in; 
  KDL::Twist bounds = KDL::Twist::Zero();
  q_init.data = Eigen::VectorXd::Zero(_ci.num_joints);

  sensor_msgs::msg::JointState js;
  js.name = _ci.joint_names;
  js.position.resize(_ci.num_joints);

  _reach_graph->vertexToWorld(_xi, _yi, _zi, x, y, z);

  // Create samples
  reachability_msgs::msg::ReachData rdata;

  std::vector<Eigen::Isometry3d> frames; 
  _reach_graph->createSphereSamplesVoxel(_xi, _yi, _zi, frames);


  for(auto frame_i : frames)
  {
    tf2::transformEigenToKDL(frame_i, p_in);

    if(_ik_solver->CartToJnt(q_init, p_in, q_out, bounds) > 0)
    {
      js.position = jntArrayToVector(q_out);
      _rco->update(js);
      if(!_rco->selfCollide())
      {
        std::vector<KDL::JntArray> sols;
        _ik_solver->getSolutions(sols);

        reachability_msgs::msg::ReachSample sample_i;
        sample_i.pose = tf2::toMsg(frame_i);
        sample_i.best_config = js.position;
        sample_i.num_configs = sols.size();

        rdata.samples.push_back(sample_i);

        } // end selfCollide
    }  // end if  ik_solver

  } // end frames_i

  // Set status 
  if(rdata.samples.size() > 0)
    rdata.state = reachability_msgs::msg::ReachData::FILLED;
  else
    rdata.state = reachability_msgs::msg::ReachData::NO_FILLED;
  
  return rdata;
}

void ReachabilityDescription::estimateReachLimits( const reachability_msgs::msg::ChainInfo &_ci)
{
  std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver;

  double max_time = 0.001;
  double eps = 1e-5;
  TRAC_IK::SolveType ik_type = TRAC_IK::SolveType::Distance;

  ik_solver.reset( new TRAC_IK::TRAC_IK(nh_, _ci.root_link, _ci.tip_link, 
                              "robot_description", 
                              max_time, eps, ik_type));

  KDL::Chain chain;
  re_->getKDLChain(_ci.root_link, _ci.tip_link, chain);
  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  double max_radius = 2.0;
  double res = 0.10;

  double xmin, ymin, zmin, xmax, ymax, zmax;
  double def_min = -1000; double def_max = 1000;
  xmin = def_max; ymin = def_max; zmin = def_max;
  xmax = def_min; ymax = def_min; zmax = def_min;

  KDL::JntArray q_init, q_out; 
  KDL::Twist bounds;
  bounds.rot = KDL::Vector(2*M_PI, 2*M_PI, 2*M_PI);
  bounds.vel = KDL::Vector(0.5*res, 0.5*res, 0.5*res);
  q_init.data = Eigen::VectorXd::Zero(_ci.num_joints);


  KDL::Frame p_in; 
  p_in = KDL::Frame::Identity();
  
  double x, y, z;
  int n = floor(2*max_radius / res);
  RCLCPP_WARN(rclcpp::get_logger("estimate"), "N: %d", n);  

  for(int xi = 0; xi < n; ++xi)
  {
  RCLCPP_WARN(rclcpp::get_logger("estimate"), "xi: %d", xi);  

    for(int yi = 0; yi < n; ++yi)
    {
      for(int zi = 0; zi < n; ++zi)
      { 
        x = -max_radius + xi*res;
        y = -max_radius + yi*res;
        z = -max_radius + zi*res;

        p_in.p = KDL::Vector(x, y, z);
        int res = ik_solver->CartToJnt(q_init, p_in, q_out, bounds);
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

  RCLCPP_WARN(rclcpp::get_logger("estimate"), "Limits: %f %f %f -- %f %f %f", xmin, ymin, zmin, xmax, ymax, zmax);  

}

/**
 * @function generateDescription 
 */
bool ReachabilityDescription::generateDescription(const std::string &_chain_group)
{
  reachability_msgs::msg::ChainInfo chain_info;
  re_->getChainInfo(_chain_group, chain_info);

  // Init reach graph
  double xmin, ymin, zmin, xmax, ymax, zmax, xmid, ymid, zmid;
  double res; 
  reachability_msgs::msg::ReachData reach_default;

  estimateReachLimits(chain_info);
  return false;
    xmin = -1.2; ymin = -1.0; zmin = -0.2;
    xmax = 1.2; ymax = 1.0; zmax = 1.6;
    res = 0.05;
    int num_voxel_samples = 32;
    reach_default.state = reachability_msgs::msg::ReachData::NO_FILLED;

    reach_graph_.reset( new ReachGraph( chain_info, xmin, ymin, zmin, xmax, ymax, zmax,
	      res, num_voxel_samples, reach_default )); 

xmid = (xmin + xmax)*0.5;
ymid = (ymin + ymax)*0.5;
zmid = (zmin + zmax)*0.5;

auto ts = std::chrono::system_clock::now();
 std::thread to1_( &ReachabilityDescription::reach_calc, this, 
                  xmin, ymin, zmin, xmid, ymid, zmax, 
                  chain_info);
 std::thread to2_( &ReachabilityDescription::reach_calc, this, 
                  xmin, ymid, zmin, xmid, ymax, zmax, 
                  chain_info);
 std::thread to3_( &ReachabilityDescription::reach_calc, this, 
                  xmid, ymin, zmin, xmax, ymid, zmax, 
                  chain_info);
 std::thread to4_( &ReachabilityDescription::reach_calc, this, 
                  xmid, ymid, zmin, xmax, ymax, zmax, 
                  chain_info);

 to1_.join();
 to2_.join();
 to3_.join();
 to4_.join();

auto tf = std::chrono::system_clock::now();
std::chrono::duration<double> dt = (tf - ts);

RCLCPP_INFO(nh_->get_logger(), "Joint Time: %f seconds ", dt.count());   

 return true;
}

/**
 * @function getReachabilityData 
 */
bool ReachabilityDescription::getReachabilityData(const double &_x, const double &_y, const double &_z, 
                                                  reachability_msgs::msg::ReachData &_data)
{
  if(!reach_graph_)
    return false;

  int xi, yi, zi;
  if(!reach_graph_->worldToVertex(_x, _y, _z, xi, yi, zi))
    return false;
  reachability_msgs::msg::ReachData rdata = reach_graph_->getState(xi, yi, zi);
 RCLCPP_WARN(rclcpp::get_logger("DEBUG"), "get reachability data: from %f %f %f to %d %d %d, samples: %d", _x, _y, _z, xi, yi, zi, rdata.samples.size());

  _data = rdata;
  return true;
}

/**
 * @function viewDescription 
 */
bool ReachabilityDescription::viewDescription()
{
  sensor_msgs::msg::PointCloud2 msg;
  msg = reach_graph_->getPCD( reachability_msgs::msg::ReachData::FILLED, 125, 0, 125);

  rclcpp::Rate r(1.0);
  for(unsigned int i = 0; i < 10; ++i)
  {
    pub_reach_->publish(msg);
    r.sleep();
    rclcpp::spin_some(nh_);
  }

  return true;
}

/**
 * @function storeDescription
 */
bool ReachabilityDescription::loadDescription(const std::string &_filename)
{
  std::string filename;
  if(_filename.size() > 0)
    filename = _filename;
  else
    filename = "/home/ana/Desktop/test_" + robot_name_;

  reachability_msgs::msg::ReachGraph msg;
  
  ReachSerial<reachability_msgs::msg::ReachGraph> reach_serial;
  bool b = reach_serial.readFromDisk(filename, msg);

  if(!b)
    return false;

  // Load ReachData with message
  reach_graph_.reset( new ReachGraph(msg));
  return true;   
}


/**
 * @function storeDescription
 */
bool ReachabilityDescription::storeDescription(const std::string &_optional_filename)
{
  std::string filename;
  if(_optional_filename.size() > 0)
    filename = _optional_filename;
  else
    filename = "/home/ana/Desktop/test_" + robot_name_;

  reachability_msgs::msg::ReachGraph msg;
  reach_graph_->toMsg(msg);
  
  ReachSerial<reachability_msgs::msg::ReachGraph> reach_serial;
  return reach_serial.writeToDisk(msg, filename);
}



}  // namespace reachability_description