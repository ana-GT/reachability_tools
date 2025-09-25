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

    //std::vector<Eigen::Isometry3d> frames;
    //rd_->getReachGraph(chain_group_)->generateSamples(0.5, 0.8, 0.8, frames);
/*
    int id = 0;
    for(auto fi : frames)
    {
      Eigen::Quaterniond q;
      q = fi.linear();
      Eigen::Vector3d z;
      z = fi.linear().col(2);
      RCLCPP_INFO(this->get_logger(), "[%d] Frame quat: %f %f %f %f --- z: %f %f %f", id, q.x(), q.y(), q.z(), q.w(), z.x(), z.y(), z.z());
      id++;
    }*/
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
  geometry_msgs::msg::PoseStamped pose = req->goal_pose;
  
  // 1. Get all reachability voxels that are at Z level and that have a reachability > min
  // if not candidates, go on checking for less desirable reachabilities
  std::vector<reachability_msgs::msg::ReachData> samples, best_samples;

  double threshold = 0.03;
  double top_percent = 0.30;
  getSamplesAtZ(pose.pose, samples, threshold);

  getSamplesRatioTop(samples, top_percent, best_samples);

  RCLCPP_INFO(logger, "Number of samples with ratio > %f: %ld, out of %ld samples at height %f", 
        top_percent, best_samples.size(), samples.size(), pose.pose.position.z);
  
    
  // 2. Identify the voxels that have a Z axis that has a rotation to the goal Z axis that is a yaw
  Eigen::Isometry3d Tf_goal, Tf_sample, Tf_ee_offset, Tf_base, Tf_ee;
  tf2::fromMsg(pose.pose, Tf_goal);
    
  tf2::fromMsg(req->grasp_offset, Tf_ee_offset);
    
  reachability_msgs::msg::ChainInfo ci_;
  rd_->getChainInfo(chain_group_, ci_);
    
    
  std::vector<reachability_msgs::msg::ReachData> aligned_samples;
  Eigen::Vector3d unit_z(0,0,1);
  Eigen::Vector3d z_ee, z_sample;
  double yaw;
  double sy, cy;
  double tx, ty;
  Tf_sample.setIdentity();

  Tf_ee = Tf_goal * Tf_ee_offset;
  z_ee = Tf_ee.linear().col(2);

  for(auto si : best_samples)
  {
     for(auto pi : si.samples)
     {
       tf2::fromMsg(pi.pose, Tf_sample);
      
       // Get the min angle between Tf_ee and sample
       z_sample = Tf_sample.linear().col(2);
       
       Eigen::Quaterniond qes; qes.setFromTwoVectors(z_sample, z_ee);
       Eigen::AngleAxisd aa(qes);
       Eigen::Matrix3d rot; rot = aa.toRotationMatrix();
       Eigen::Vector3d ypr; ypr = rot.eulerAngles(2,1,0);
       
       double acos = unit_z.dot(aa.axis());
       
        
       if( fabs(acos) > 0.9 && fabs(Tf_sample.translation()(1)) < 0.3 )
       {
          yaw = acos > 0.0? aa.angle() : -1*aa.angle();
          sy = sin(yaw); cy = cos(yaw);
          tx = Tf_ee.translation()(0) - (cy * Tf_sample.translation()(0) - sy * Tf_sample.translation()(1) );
          ty = Tf_ee.translation()(1) - (sy * Tf_sample.translation()(0) + cy * Tf_sample.translation()(1) );          
       
          double pir = 180.0/3.1416;
          RCLCPP_INFO(this->get_logger(), "X: %f y: %f z: %f. YPR: %f %f %f", pi.pose.position.x, pi.pose.position.y, pi.pose.position.z, ypr(2)*pir, ypr(1)*pir, ypr(0)*pir);

         RCLCPP_INFO(this->get_logger(), "Sample yaw pose: x, y with small z: : %.3f %.3f %.3f yaw: %.3f AXIS: %.3f %.3f %.3f",  Tf_base.translation()(0),  Tf_base.translation()(1),  Tf_base.translation()(2), aa.angle()*pir, aa.axis()(0), aa.axis()(1), aa.axis()(2));
         
           // Get start guess for IK
           Eigen::Isometry3d Tf_init;
           Tf_init.setIdentity();
           Tf_init.translation() << tx, ty, 0;
           Tf_init.linear() = Eigen::AngleAxisd( yaw, Eigen::Vector3d(0,0,1)).toRotationMatrix();

           reachability_msgs::msg::MobilePose sol;
           sol.arm_config = vectorToJointState(pi.best_config, ci_);
           sol.base_pose.pose = tf2::toMsg(Tf_init); // Tf_init
           sol.base_pose.header.frame_id = "world";
           res->solutions.push_back(sol);
       
         } // if fabs

     } // for pi

  }



  
  res->success = res->solutions.empty()? false : true;

  
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

