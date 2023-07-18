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
bool ReachabilityDescription::initialize(const double &_max_time, 
                                         const double &_eps, 
                                         const TRAC_IK::SolveType &_ik_type)
{

  if(!nh_->has_parameter("robot_description"))
    nh_->declare_parameter("robot_description", std::string(""));

  if(!nh_->has_parameter("robot_description_semantic"))
    nh_->declare_parameter("robot_description_semantic", std::string(""));

  nh_->get_parameter("robot_description", urdf_string_);
  nh_->get_parameter("robot_description_semantic", srdf_string_);

    re_.reset(new RobotEntity());
    if(!re_->init(urdf_string_, srdf_string_))
        return false;

    rco_.reset(new robot_unit::RobotCollisionObject());
    if(!rco_->init("world", "panda", urdf_string_, srdf_string_))
        return false;

    pub_reach_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("reach_data_test", 10);

    return true;
}

KDL::Frame makeKDLFrame(double x, double y, double z, double roll, double pitch, double yaw)
{
    KDL::Frame pi;
    pi.p = KDL::Vector(x, y, z);
    pi.M = KDL::Rotation::EulerZYX(yaw, pitch, roll);
    return pi;
}

bool ReachabilityDescription::quickTest()
{
    RCLCPP_WARN(nh_->get_logger(), "QUICK TEEEEEEEEEST" );

    // Set rco_ to all zeros and check self-collision
    sensor_msgs::msg::JointState js;
    std::vector<std::string> jn = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    std::vector<double> jv = {0, 0, 0, 0, 0, 0, 0};
    js.name = jn;
    js.position = jv;
    rco_->update(js);
    bool b = rco_->selfCollide();
    RCLCPP_WARN(nh_->get_logger(), "Self collide for all zeros: %d !!!!!!!!!!!!!!!!!!!!", b );

    std::vector<double> jv2 = {0, 0, 0, -1.57, 0, 1.57, 0};
    js.name = jn;
    js.position = jv2;
    rco_->update(js);
    b = rco_->selfCollide();
    RCLCPP_WARN(nh_->get_logger(), "Self collide for good pose: %d !!!!!!!!!!!!!!!!!!!!", b );


    return true;
}

/**
 *  
 */
void ReachabilityDescription::reach_calc( const double &_min_x, const double &_min_y, const double &_min_z,
                                          const double &_max_x, const double &_max_y, const double &_max_z,
                                          const std::string &_base_link, const std::string &_tip_link)
{
    int found_sols = 0;
    int found_in_coll = 0;

    double max_time = 0.001;
    double eps = 1e-5;
    TRAC_IK::SolveType ik_type = TRAC_IK::SolveType::Distance;

    TRAC_IK::TRAC_IK ik_solver(nh_, _base_link, _tip_link, "robot_description", max_time, eps, ik_type);

    ReachData reach_default;
    reach_default.state = ReachDataState::NO_FILLED;

 sensor_msgs::msg::JointState js;
 std::vector<std::string> jn = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
 js.name = jn;
 std::vector<double> jv = {0, 0, 0, 0, 0, 0, 0};

double x, y, z;
  KDL::JntArray q_init, q_out; 
  KDL::Frame p_in; 
  KDL::Twist bounds = KDL::Twist::Zero();
  q_init.data = Eigen::VectorXd::Zero(7);

    ReachGraph reach_graph( _min_x, _min_y, _min_z, _max_x, _max_y, _max_z,
  	                        reach_graph_->getResolution(), reach_default ); 

    robot_unit::RobotCollisionObject rco;
    if(!rco.init("world", "panda", urdf_string_, srdf_string_))
        return;

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
            for(int i = 0; i < jv.size(); ++i)
                jv[i] = q_out(i);
            js.position = jv;
            rco.update(js);
            if(!rco.selfCollide())
            {
              //reach_graph.setState(xi, yi, zi, rd_filled);
              found_sols++;
            }
            else
              found_in_coll++;

          }  
        } // for zi
    } // for yi
 } // for xi 

RCLCPP_WARN(rclcpp::get_logger("reach"), "Done in thread! Sols: %u/%u (found in coll: %u). xyz min: (%.3f, %.3f, %.3f) xyz max: (%.3f, %.3f, %.3f)", 
            found_sols, reach_graph.getNumPoints(), found_in_coll,_min_x, _min_y, _min_z, _max_x, _max_y, _max_z);
}

/**
 * @function generateDescription 
 */
bool ReachabilityDescription::generateDescription(const std::string &_chain_group)
{
  std::string base_link, tip_link;
  re_->getChainInfo(_chain_group, base_link, tip_link);

  // Init reach graph
  double xmin, ymin, zmin, xmax, ymax, zmax, xmid, ymid, zmid;
  double res; ReachData reach_default;

    xmin = -1.0; ymin = -1.0; zmin = -1.0;
    xmax = 1.0; ymax = 1.0; zmax = 1.0;
    res = 0.05;
    reach_default.state = ReachDataState::NO_FILLED;

    reach_graph_.reset( new ReachGraph( xmin, ymin, zmin, xmax, ymax, zmax,
	      res, reach_default )); 

xmid = (xmin + xmax)*0.5;
ymid = (ymin + ymax)*0.5;
zmid = (zmin + zmax)*0.5;

RCLCPP_INFO(nh_->get_logger(), "Thread start!");

auto ts = std::chrono::system_clock::now();

 std::thread to1_( &ReachabilityDescription::reach_calc, this, xmin, ymin, zmin, xmid, ymid, zmax, base_link, tip_link );
 std::thread to2_( &ReachabilityDescription::reach_calc, this, xmin, ymid, zmin, xmid, ymax, zmax, base_link, tip_link );
 std::thread to3_( &ReachabilityDescription::reach_calc, this, xmid, ymin, zmin, xmax, ymid, zmax, base_link, tip_link );
 std::thread to4_( &ReachabilityDescription::reach_calc, this, xmid, ymid, zmin, xmax, ymax, zmax, base_link, tip_link );

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