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
    std::string urdf_string, srdf_string, chain_group, base_link, tip_link;


  if(!nh_->has_parameter("robot_description"))
    nh_->declare_parameter("robot_description", std::string(""));

  if(!nh_->has_parameter("robot_description_semantic"))
    nh_->declare_parameter("robot_description_semantic", std::string(""));

  if(!nh_->has_parameter("group"))
    nh_->declare_parameter("group", std::string(""));

  nh_->get_parameter("robot_description", urdf_string);
  nh_->get_parameter("robot_description_semantic", srdf_string);
  nh_->get_parameter("group", chain_group);


    re_.reset(new RobotEntity());
    if(!re_->init(urdf_string, srdf_string))
        return false;

    re_->getChainInfo(chain_group, base_link, tip_link);

    ik_solver_.reset( new TRAC_IK::TRAC_IK(nh_, base_link, tip_link, "robot_description", _max_time, _eps, _ik_type));

    rco_.reset(new robot_unit::RobotCollisionObject());
    if(!rco_->init("world", "panda", urdf_string, srdf_string))
        return false;

    // Init reach graph
    double min_x, min_y, min_z, max_x, max_y, max_z; double res; 
    ReachData reach_default;

    min_x = -1.0; min_y = -1.0; min_z = -1.0;
    max_x = 1.0; max_y = 1.0; max_z = 1.0;
    res = 0.05;
    reach_default.state = ReachDataState::NO_FILLED;

    reach_graph_.reset( new ReachGraph( min_x, min_y, min_z, max_x, max_y, max_z,
	      res, reach_default )); 

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
void ReachabilityDescription::reach_calc()
{
    int sol = 0;
    std::string base_link = "panda_link0";
    std::string tip_link = "panda_hand_tcp";
    double max_time = 0.001;
    double eps = 1e-5;
    TRAC_IK::SolveType ik_type = TRAC_IK::SolveType::Distance;

    TRAC_IK::TRAC_IK ik_solver(nh_, base_link, tip_link, "robot_description", max_time, eps, ik_type);

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    double res;
    
    ReachData reach_default;
    reach_default.state = ReachDataState::NO_FILLED;

    min_x = -0.3; min_y = -0.3; min_z = -0.3;
    max_x = 0.3; max_y = 0.3; max_z = 0.3;
    res = 0.05;

 sensor_msgs::msg::JointState js;
 std::vector<std::string> jn = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
 js.name = jn;
 std::vector<double> jv = {0, 0, 0, 0, 0, 0, 0};

double x, y, z;
  KDL::JntArray q_init, q_out; 
  KDL::Frame p_in; 
  KDL::Twist bounds = KDL::Twist::Zero();
  q_init.data = Eigen::VectorXd::Zero(7);

    ReachGraph reach_graph( min_x, min_y, min_z, max_x, max_y, max_z,
  	                        res, reach_default ); 
clock_t ts, tf; double dt;
ts = clock();
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
            sol++;
            /*rco_->update(js);
            if(!rco_->selfCollide())
            {
              reach_graph_->setState(xi, yi, zi, rd_filled);
              found_sols++;
            }
            else
              found_in_coll++;*/
          }  
        } // for zi
    } // for yi
 } // for xi 
tf = clock();
dt = (double)(tf-ts)/(double)CLOCKS_PER_SEC;
RCLCPP_WARN(rclcpp::get_logger("neil"), "Done in thread! Sols: %u out of %u. Time: %f seconds", sol, reach_graph.getNumX()*reach_graph.getNumY()*reach_graph.getNumZ(), dt);
}

/**
 * @function generateDescription 
 */
bool ReachabilityDescription::generateDescription()
{
RCLCPP_INFO(nh_->get_logger(), "generateDescription start..."); 

  KDL::JntArray q_init, q_out; 
  KDL::Frame p_in; 
  KDL::Twist bounds = KDL::Twist::Zero();

 int found_sols = 0;
 int found_in_coll = 0;
  q_init.data = Eigen::VectorXd::Zero(7);

 double x, y, z;
 ReachData rd_filled;
 rd_filled.state = ReachDataState::FILLED;

 ReachData rd_self_coll;
 rd_self_coll.state = ReachDataState::COLLISION;


 // Check for self-collision
 sensor_msgs::msg::JointState js;
 std::vector<std::string> jn = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
 js.name = jn;
 std::vector<double> jv = {0, 0, 0, 0, 0, 0, 0};
 

 std::thread to1_( &ReachabilityDescription::reach_calc, this );
 std::thread to2_( &ReachabilityDescription::reach_calc, this );
 std::thread to3_( &ReachabilityDescription::reach_calc, this );
 std::thread to4_( &ReachabilityDescription::reach_calc, this );

 to1_.join();
 to2_.join();
 to3_.join();
to4_.join();

 clock_t ts, tf; double dt;
 ts = clock();
 for(int xi = 0; xi < reach_graph_->getNumX(); ++xi )
 {
    for(int yi = 0; yi < reach_graph_->getNumY(); ++yi)
    {
        for(int zi = 0; zi < reach_graph_->getNumZ(); ++zi)
        {
          reach_graph_->vertexToWorld(xi, yi, zi, x, y, z);
          p_in = makeKDLFrame(x, y, z, 0, 0, 0);  
          if(ik_solver_->CartToJnt(q_init, p_in, q_out, bounds) > 0)
          {
            for(int i = 0; i < jv.size(); ++i)
                jv[i] = q_out(i);
            js.position = jv;
            /*rco_->update(js);
            if(!rco_->selfCollide())
            {
              reach_graph_->setState(xi, yi, zi, rd_filled);
              found_sols++;
            }
            else
              found_in_coll++;*/
          }  
        } // for zi
    } // for yi
 } // for xi

sensor_msgs::msg::PointCloud2 msg;
 tf = clock();
 dt = (double)(tf-ts)/(double)CLOCKS_PER_SEC;
 
RCLCPP_INFO(nh_->get_logger(), "Found %u solutions (with %u in self-collision) out of %u trials. Time: %f seconds ", 
            found_sols, found_in_coll, reach_graph_->getNumPoints(), dt);   
 

msg = reach_graph_->getPCD(ReachDataState::FILLED, 125, 0, 125);
pub_reach_->publish(msg);

 return true;
}


}  // namespace reachability_description