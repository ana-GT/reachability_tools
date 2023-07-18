#include <reachability_description/reachability_description.h>

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
    ChainInfo chain_info;
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
                                          const ChainInfo &_ci)
{
    int found_sols = 0;
    int found_in_coll = 0;

    double max_time = 0.001;
    double eps = 1e-5;
    TRAC_IK::SolveType ik_type = TRAC_IK::SolveType::Distance;

    TRAC_IK::TRAC_IK ik_solver(nh_, _ci.root_link, _ci.tip_link, 
                              "robot_description", 
                              max_time, eps, ik_type);

    ReachData reach_default;
    reach_default.state = ReachDataState::NO_FILLED;

 sensor_msgs::msg::JointState js;
 js.name = _ci.joint_names;
 js.position.resize(_ci.num_joints);

  double x, y, z;
  KDL::JntArray q_init, q_out; 
  KDL::Frame p_in; 
  KDL::Twist bounds = KDL::Twist::Zero();
  q_init.data = Eigen::VectorXd::Zero(_ci.num_joints);

    ReachGraph reach_graph( _ci, _min_x, _min_y, _min_z, _max_x, _max_y, _max_z,
  	                        reach_graph_->getResolution(), reach_default ); 

    robot_unit::RobotCollisionObject rco;
    if(!rco.init(DEFAULT_REF_FRAME, robot_name_, urdf_string_, srdf_string_))
        return;

    ReachData rd_filled;
    rd_filled.state = ReachDataState::FILLED;

for(int xi = 0; xi < reach_graph.getNumX(); ++xi )
 {
    for(int yi = 0; yi < reach_graph.getNumY(); ++yi)
    {
        for(int zi = 0; zi < reach_graph.getNumZ(); ++zi)
        {
          reach_graph.vertexToWorld(xi, yi, zi, x, y, z);
          p_in = makeKDLFrame(x, y, z, 0, 0, 0);  
          if(ik_solver.CartToJnt(q_init, p_in, q_out, bounds) > 0)
          {
            js.position = jntArrayToVector(q_out);
            rco.update(js);
            if(!rco.selfCollide())
            {
              std::vector<KDL::JntArray> sols;
              ik_solver.getSolutions(sols);
              rd_filled.num_sols = sols.size();
              reach_graph.setState(xi, yi, zi, rd_filled);
              found_sols++;
            }
            else
              found_in_coll++;
          }  
        } // for zi
    } // for yi
 } // for xi 

// Fill reach
reach_fill_mutex_.lock();
for(int i = 0; i < reach_graph.getNumPoints(); ++i)
{
  int xli, yli, zli, xi, yi, zi;
  double x, y, z;

  reach_graph.indexToVertex(i, xli, yli, zli);
  ReachData data = reach_graph.getState(i);

  // Transform index to double then back to index of main reach
  reach_graph.vertexToWorld(xli, yli, zli, x, y, z);

  reach_graph_->worldToVertex(x, y, z, xi, yi, zi);
  reach_graph_->setState(xi, yi, zi, data);
}
reach_fill_mutex_.unlock();

RCLCPP_WARN(rclcpp::get_logger("reach"), "Done in thread! Sols: %u/%u (found in coll: %u). xyz min: (%.3f, %.3f, %.3f) xyz max: (%.3f, %.3f, %.3f)", 
            found_sols, reach_graph.getNumPoints(), 
            found_in_coll,_min_x, _min_y, _min_z, _max_x, _max_y, _max_z);
}

/**
 * @function generateDescription 
 */
bool ReachabilityDescription::generateDescription(const std::string &_chain_group)
{
  ChainInfo chain_info;
  re_->getChainInfo(_chain_group, chain_info);

  // Init reach graph
  double xmin, ymin, zmin, xmax, ymax, zmax, xmid, ymid, zmid;
  double res; ReachData reach_default;

    xmin = -1.0; ymin = -1.0; zmin = -1.0;
    xmax = 1.0; ymax = 1.0; zmax = 1.0;
    res = 0.05;
    reach_default.state = ReachDataState::NO_FILLED;

    reach_graph_.reset( new ReachGraph( chain_info, xmin, ymin, zmin, xmax, ymax, zmax,
	      res, reach_default )); 

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

sensor_msgs::msg::PointCloud2 msg;
msg = reach_graph_->getPCD(ReachDataState::FILLED, 125, 0, 125);
pub_reach_->publish(msg);

 return true;
}


}  // namespace reachability_description