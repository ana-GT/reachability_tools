/**
 * @file simple_placement_solver.cpp
 */
#include <reachability_applications/simple_mobile_pose_node.h>

#include <algorithm>
#include <cfloat>
#include <nlopt.h>

#include <reachability_description/reach_utilities.h>
#include <reachability_description/reach_graph.h>

using namespace std::chrono_literals;

const auto logger = rclcpp::get_logger("simple_reachability_query");

/**
 * Constructor
 */
SimpleMobilePose::SimpleMobilePose() :
rclcpp::Node("simple_mobile_pose")
{
    this->declare_parameter("chain_group_name", std::string(""));
    this->declare_parameter("robot_name", std::string(""));
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

    std::vector<Eigen::Isometry3d> frames;
    rd_->getReachGraph(chain_group_)->generateSamples(0.5, 0.8, 0.8, frames);

    int id = 0;
    for(auto fi : frames)
    {
      Eigen::Quaterniond q;
      q = fi.linear();
      Eigen::Vector3d z;
      z = fi.linear().col(2);
      RCLCPP_INFO(this->get_logger(), "[%d] Frame quat: %f %f %f %f --- z: %f %f %f", id, q.x(), q.y(), q.z(), q.w(), z.x(), z.y(), z.z());
      id++;
    }
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
  RCLCPP_INFO(logger, "!!!!!!Received service to query mobile poses for this pose");

  // 0. Transform the pose to the root link frame
  geometry_msgs::msg::PoseStamped pose = req->goal_pose;
  
  // 1. Get all reachability voxels that are at Z level and that have a reachability > min
  // if not candidates, go on checking for less desirable reachabilities
  std::vector<reachability_msgs::msg::ReachData> samples, best_samples;

  double threshold = 0.03;
  double top_percent = 0.3;
  getSamplesAtZ(pose.pose, samples, threshold);

  getSamplesRatioTop(samples, top_percent, best_samples);

  RCLCPP_INFO(logger, "Number of samples with ratio > %f: %ld, out of %ld samples at height %f", 
        top_percent, best_samples.size(), samples.size(), pose.pose.position.z);
  
  // 2. Identify the voxels that have a Z axis that has a rotation to the goal Z axis that is a yaw
  Eigen::Isometry3d Tf_goal, Tf_sample;
  tf2::fromMsg(pose.pose, Tf_goal);
  
  std::vector<reachability_msgs::msg::ReachData> aligned_samples;

  for(auto si : best_samples)
  {
     reachability_msgs::msg::ReachData ri;

     for(auto pi : si.samples)
     {
      tf2::fromMsg(pi.pose, Tf_sample);
      Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Tf_goal.linear().col(2), Tf_sample.linear().col(2));
      Eigen::AngleAxisd aa(q);
      Eigen::Vector3d unit_z(0,0,1);

      if(  fabs(unit_z.dot( aa.axis() )) > 0.86 ) // cos(30), cos(20) = 0.94
      {
        ri.samples.push_back(pi);
      } 
     } // for pi

     if(!ri.samples.empty())
     {
       aligned_samples.push_back(ri);       
       RCLCPP_INFO(this->get_logger(), "Samples aligned: %ld / %ld", ri.samples.size(), si.samples.size());
     }
  }

  // 3. From the ones above, order according to # of solutions per voxel (to withstand inaccuracy)
  
  // 4. Per each voxel, get the mean orientation of the solutions
  
  // 5. Calculate the rotation + translation of the goal pose to the voxel (location + rotation)
  
  // 6. Apply the inverse transform to the base
  
  // 7. Calculate the Tf base
  
  // 8. Get K-means of the base locations
  
  // 9. Use the means as seeds for the IK problem of base + arm. Use as start arm config seed the value from rd if required
  
  // 10. Return solutions

  // Get X,Y,Z
  /*
  double x, y, z;
  x = req->bbox.center.position.x;
  y = req->bbox.center.position.y;
  z = req->bbox.center.position.z;
  
  // Fill the voxel for this location
  reachability_msgs::msg::ReachData reach_data;
  reach_data = rd_->calculateReachabilityPoint(x, y, z, chain_group_);
    
  // Return the EE poses / joint states
  reachability_msgs::msg::ChainInfo ci;
  rd_->getChainInfo(chain_group_, ci);
  
  for(auto si : reach_data.samples)
  {
    geometry_msgs::msg::PoseStamped ps;
    sensor_msgs::msg::JointState js;
    
    ps.pose = si.pose;
    js = vectorToJointState(si.best_config, ci);
    
    res->ee_poses.push_back(ps);
    res->joint_states.push_back(js);
  }

  res->success = res->ee_poses.empty()? false : true; */
}


/**
 * @function getTransform // (-0.062, 0.0, 0.291);
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

