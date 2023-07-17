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
      RCLCPP_INFO(nh_->get_logger(), "Initialize");

    std::string urdf_string, srdf_string, chain_group, base_link, tip_link;

      RCLCPP_INFO(nh_->get_logger(), "Declaring...");

  if(!nh_->has_parameter("robot_description"))
    nh_->declare_parameter("robot_description", std::string(""));

  if(!nh_->has_parameter("robot_description_semantic"))
    nh_->declare_parameter("robot_description_semantic", std::string(""));

  if(!nh_->has_parameter("group"))
    nh_->declare_parameter("group", std::string(""));

      RCLCPP_INFO(nh_->get_logger(), "Getting params...");


  nh_->get_parameter("robot_description", urdf_string);
  nh_->get_parameter("robot_description_semantic", srdf_string);
  nh_->get_parameter("group", chain_group);

      RCLCPP_INFO(nh_->get_logger(), "Resetting urdf");

  urdf_.reset(new urdf::Model());

      RCLCPP_INFO(nh_->get_logger(), "Init urdf");

  if(!urdf_->initString(urdf_string))
    return false;

      RCLCPP_INFO(nh_->get_logger(), "Resetting srdf");


  srdf_model_.reset(new srdf::Model());
  if(!srdf_model_->initString(*urdf_, srdf_string))
    return false;

  RCLCPP_INFO(nh_->get_logger(), "Got group %s ", chain_group.c_str());


  std::vector<srdf::Model::Group> groups = srdf_model_->getGroups();
  bool found_chain = false;
  for(int i = 0; i < groups.size(); ++i)
  {
    if(groups[i].name_ == chain_group)
    {
        if(groups[i].chains_.size() == 1)
        {
         printf("Group %s has chains of size %lu \n", chain_group.c_str(), groups[i].chains_.size());
         base_link = groups[i].chains_[0].first;
         tip_link = groups[i].chains_[0].second;
        RCLCPP_INFO(nh_->get_logger(), "Using base %s and tip %s ", base_link.c_str(), tip_link.c_str());   
         found_chain = true;
         break;
        }
    }
  }

  if(!found_chain)
    return false;

    ik_solver_.reset( new TRAC_IK::TRAC_IK(nh_, base_link, tip_link, "robot_description", _max_time, _eps, _ik_type));

    rco_.reset(new robot_unit::RobotCollisionObject());
    if(!rco_->init("world", "panda", urdf_string, srdf_string))
        return false;

    return true;
}

KDL::Frame makeKDLFrame(double x, double y, double z, double roll, double pitch, double yaw)
{
    KDL::Frame pi;
    pi.p = KDL::Vector(x, y, z);
    pi.M = KDL::Rotation::EulerZYX(yaw, pitch, roll);
    return pi;
}

/**
 * @function generateDescription 
 */
bool ReachabilityDescription::generateDescription()
{
 double x_min, x_max;
 x_min = 0.5; x_max = 0.7;
 double y_min, y_max;
 y_min = -0.5; y_max = 0.5;
 double z_min, z_max;
 z_min = 0.2; z_max = 1.0;

  double d_linear = 0.05;

 int num_x, num_y, num_z;

 num_x = ceil((x_max - x_min)/d_linear);
 num_y = ceil((y_max - y_min)/d_linear); 
 num_z = ceil((z_max - z_min)/d_linear);

  KDL::JntArray q_init, q_out; 
  KDL::Frame p_in; 
  KDL::Twist bounds = KDL::Twist::Zero();

 int found_sols = 0;
 int trials = num_x*num_y*num_z;

  q_init.data = Eigen::VectorXd::Zero(7);

 double x, y, z;
 clock_t ts, tf; double dt;
 ts = clock();
 for(int xi = 0; xi < num_x; ++xi )
 {
    x = x_min + d_linear*xi;

    for(int yi = 0; yi < num_y; ++yi)
    {
          y = y_min + d_linear*yi;

        for(int zi = 0; zi < num_z; ++zi)
        {
          z = z_min + d_linear*zi;

          p_in = makeKDLFrame(x, y, z, 0, 0, 0);  
          if(ik_solver_->CartToJnt(q_init, p_in, q_out, bounds) > 0)
            found_sols++;
        }
    }
 }
 tf = clock();
 dt = (double)(tf-ts)/(double)CLOCKS_PER_SEC;
 
RCLCPP_INFO(nh_->get_logger(), "Found %u solutions out of %u trials. Time: %f seconds ", found_sols, trials, dt);   
 
 return true;
}


}  // namespace reachability_description