/**
 * @file simple_placement_solver.cpp
 */
#include <reachability_applications/simple_mobile_pose_node.h>

#include <algorithm>
#include <cfloat>
#include <nlopt.h>

#include <reachability_description/reach_utilities.h>
#include <reachability_description/reach_graph.h>
#include <reachability_description/geom_se2.h>

const auto logger = rclcpp::get_logger("simple_reachability_query");

/**
 * Constructor
 */
SimpleMobilePose::SimpleMobilePose() :
rclcpp::Node("simple_mobile_pose")
{
    this->declare_parameter("chain_group_name", std::string(""));
    this->declare_parameter("robot_name", std::string(""));
    
    rclcpp::QoS qos_latch(1);
    qos_latch.transient_local();
    pub_base_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("debug_base_poses", qos_latch);
}

/*s
 * Initialize
 */
bool SimpleMobilePose::initialize()
{
      RCLCPP_INFO(this->get_logger(), "Initializing SimpleMobilePose");   

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

     this->get_parameter("chain_group_name", chain_group_);
     
     if(!this->get_parameter("robot_name", robot_name_))
       return false;
       
    // Init Reachability
    rd_.reset( new reachability_description::ReachabilityDescription(this->shared_from_this()));
    
    if (!rd_->initialize(robot_name_))
      return false;
      
    if (!rd_->initializeGroup(chain_group_))
      return false;

    if(!rd_->loadDescription(chain_group_))
    {
      RCLCPP_INFO(this->get_logger(), "Error loading description from file");   
      return false;
    }


    // Get graph
    rd_->getReachGraph(chain_group_)->toMsg(rg_); 
    
    if(!reach_utils::getMinMaxSamples(rg_, min_samples_, max_samples_))
      return false;

      RCLCPP_INFO(this->get_logger(), "Initializing SimpleMobilePose-- ended fine");   

    rd_->viewDescription(chain_group_);

    // Initialize IK plugin with different costs
    pluginlib::ClassLoader<RachOptimizer> rach_loader("rach_ik", "RachOptimizer");

    try
    {
      ro_ = rach_loader.createSharedInstance("rach_ik_plugins::MobileHeuristicOptimizer");
      if (!ro_->init())
      	return false;
    }
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(logger,
      		"The plugin failed to load for some reason. Error: %s\n", 
      		ex.what());
      return false;
    }
    ro_->loadKinematics(chain_group_);

    return true;
}


/**
 * Offer service to get pose
 */
bool SimpleMobilePose::setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = this->create_service<reachability_msgs::srv::GetMobilePoses>("get_mobile_poses", 
                std::bind(&SimpleMobilePose::handleSrv, this, _1, _2));

    return true;
}

bool SimpleMobilePose::getSamplesAtZ(const geometry_msgs::msg::Pose &_pose,
  std::vector<reachability_msgs::msg::ReachData> &_psz,
  const double &_threshold)
{
  _psz.clear();

  double ref_z = _pose.position.z;

  for(auto pi : rg_.points)
  {
    if(pi.samples.empty())
      continue;

    double z = pi.samples[0].pose.position.z;
    if( fabs(z - ref_z) < _threshold )
      _psz.push_back(pi);
  }

  return !_psz.empty();
}

bool SimpleMobilePose::getSamplesRatioTop( const std::vector<reachability_msgs::msg::ReachData> &_samples, 
          const double &_ratio,   
          std::vector<reachability_msgs::msg::ReachData> &_best_samples)
{
  for(auto si : _samples)
  {
    auto ri = (double) si.samples.size() / (double) max_samples_; 
    if(ri >= (1.0 -_ratio))
      _best_samples.push_back(si);
  }

  return !_best_samples.empty();
}


/**
 * Handle service
 */
void SimpleMobilePose::handleSrv(const std::shared_ptr<reachability_msgs::srv::GetMobilePoses::Request> req,
               std::shared_ptr<reachability_msgs::srv::GetMobilePoses::Response> res)
{
  RCLCPP_INFO(logger, "Received service to query mobile poses for this pose");

  // 0. Transform the pose to the root link frame  
  // 1. Get all reachability voxels that are at Z level and that have a reachability > min
  // if not candidates, go on checking for less desirable reachabilities
  std::vector<reachability_msgs::msg::ReachData> samples, best_samples;

  double threshold = 0.03;
  double top_percent = 0.30;
  getSamplesAtZ(req->goal_pose.pose, samples, threshold);
  getSamplesRatioTop(samples, top_percent, best_samples);

  RCLCPP_INFO(logger, "Number of samples with ratio > %f: %ld, out of %ld samples at height %f", 
        top_percent, best_samples.size(), samples.size(), req->goal_pose.pose.position.z);
      
  // 2. Identify the voxels that have a Z axis that has a rotation to the goal Z axis that is a yaw
  Eigen::Isometry3d Tf_goal, Tf_sample, Tf_ee_offset, Tf_base, Tf_ee;

  tf2::fromMsg(req->goal_pose.pose, Tf_goal);    
  tf2::fromMsg(req->grasp_offset, Tf_ee_offset);
  Tf_ee = Tf_goal * Tf_ee_offset;
    
  reachability_msgs::msg::ChainInfo ci_;
  rd_->getChainInfo(chain_group_, ci_);
        
  std::vector<reachability_msgs::msg::ReachData> aligned_samples;
  double tx, ty, yaw;
  double disc_thresh;
  
  disc_thresh = 30.0*3.1416/180.0;
  
  geometry_msgs::msg::PoseArray poses_debug;
  std::vector<Eigen::Isometry3d> poses_tfs;
  for(auto si : best_samples)
  {
     for(auto pi : si.samples)
     {
       tf2::fromMsg(pi.pose, Tf_sample);
      
       if( reach_utils::isApproxPlanarTransform(Tf_sample, Tf_ee, tx, ty, yaw, disc_thresh))
       {           
           Eigen::Isometry3d Tf_base;
           Tf_base = reach_utils::getPlanarTransform(tx, ty, yaw);           
           poses_tfs.push_back(Tf_base);
           poses_debug.poses.push_back(tf2::toMsg(Tf_base));
       }

     } // for pi
  } // for si
  
  // Calculate KMedoids
  se2::KMedoids km;
  km.addPoints(poses_tfs);
  int k = 8;
  std::vector<Eigen::Isometry3d> u;
  std::vector<unsigned int> indices;

  sensor_msgs::msg::JointState js_init;
  sensor_msgs::msg::JointState js_sol;
  geometry_msgs::msg::PoseStamped msg_ee;
  
  msg_ee.pose = tf2::toMsg(Tf_ee);
  msg_ee.header.frame_id = "world";
  geometry_msgs::msg::PoseStamped msg_base_init, msg_base_sol;
  
  if(km.kmedoids(k, u, indices))
  { 
     for(auto ui : u)
     {
        msg_base_init.pose = tf2::toMsg(ui);
        auto pi = ui.translation();
        msg_base_init.header.frame_id = "world";

        // Get start guess for IK 
        ro_->getMobileConfiguration( chain_group_,
                msg_ee,
                js_init, msg_base_init,
                js_sol, msg_base_sol);
        RCLCPP_INFO(logger,"Guess start: %f %f %f, ended up with: %f %f %f", pi(0), pi(1), pi(2), msg_base_sol.pose.position.x, msg_base_sol.pose.position.y, msg_base_sol.pose.position.z);
        reachability_msgs::msg::MobilePose sol;
        sol.arm_config = js_sol; //vectorToJointState(pi.best_config, ci_);
        sol.base_pose = msg_base_sol;
        res->solutions.push_back(sol);
     }  
     
  } // if
  
  // Publish all poses debug
  poses_debug.header.frame_id = "world";
  poses_debug.header.stamp = this->now();
  pub_base_poses_->publish(poses_debug);
  
    
  res->success = res->solutions.empty()? false : true;  
}


/**
 * @function getTransform
 */
bool SimpleMobilePose::getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx)
{
   geometry_msgs::msg::TransformStamped tfxs;
   try
   {
      tfxs = tf_buffer_->lookupTransform(_source, _target, rclcpp::Time(0), rclcpp::Duration(1, 0));
   }
   catch (tf2::TransformException& ex)
   {
      RCLCPP_ERROR_STREAM(logger, "No transform from " << _source << " to " << _target
                                                       << ".  Error: " << ex.what());
      return false;
   }
   
   _Tfx = tf2::transformToEigen(tfxs);
   return true;
}


////////////////////////////////////

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   std::shared_ptr<SimpleMobilePose> rtt = std::make_shared<SimpleMobilePose>();

  if(!rtt->initialize())
    return 1;

  // Offer service
  rtt->setServices();

  rclcpp::spin(rtt);
  rclcpp::shutdown();
  return 0;    
}

