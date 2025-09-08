/**
 * @file simple_get_reach_data_node.cpp
 */
#include <reachability_applications/simple_get_reach_data_node.h>

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
SimpleGetReachData::SimpleGetReachData() :
rclcpp::Node("simple_get_reach_data")
{
    this->declare_parameter("chain_group_name", std::string(""));
    this->declare_parameter("robot_name", std::string(""));
}

/*s
 * Initialize
 */
bool SimpleGetReachData::initialize()
{
      RCLCPP_INFO(this->get_logger(), "Initializing SimpleGetReachData");   

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

      RCLCPP_INFO(this->get_logger(), "Initializing SimpleGetReachData-- ended fine");   

    rd_->viewDescription(chain_group_);

    return true;
}


/**
 * Offer service to get pose
 */
bool SimpleGetReachData::setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = this->create_service<reachability_msgs::srv::GetReachData>("get_reach_data", 
                std::bind(&SimpleGetReachData::handleSrv, this, _1, _2));

    return true;
}


/**
 * Handle service
 */
void SimpleGetReachData::handleSrv(const std::shared_ptr<reachability_msgs::srv::GetReachData::Request> req,
               std::shared_ptr<reachability_msgs::srv::GetReachData::Response> res)
{
  RCLCPP_INFO(logger, "!!!!!!Received service to query reach data for this pose");

  // 0. Transform the pose to the root link frame
  geometry_msgs::msg::PoseStamped pose = req->goal_pose;
  
  // 1. Get the reachability voxel corresponding to this pose
  reachability_msgs::msg::ReachData rdata;

  if(!rd_->getReachabilityData(chain_group_, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, rdata))
  {  RCLCPP_ERROR(logger, "!!!!!!getReachData returned false oh no");
    res->success = false;
    return;
  }    
  // 2. Get the best sample according to metric
  double max_val = 0.0;
  int max_id = -1;
  
  int id = 0;
  for(auto si : rdata.samples)
  { RCLCPP_WARN(logger, "Sample metric 1 and 2: %f and %f", si.metrics[0].value, si.metrics[1].value);
    if(si.metrics[0].value > max_val)
    {
      max_id = id;
      max_val = si.metrics[0].value;
    }
    
    id++;
  }
  RCLCPP_ERROR(logger, "Max id: %d, max_val: %f", max_id, max_val);
  if(max_id < 0)
  {
    res->success = false;
    return;  
  }
  
  // 3. Return the sample
  reachability_msgs::msg::ChainInfo ci;
  rd_->getChainInfo(chain_group_, ci);
  
  sensor_msgs::msg::JointState js;
  js = vectorToJointState(rdata.samples[max_id].best_config, ci);
  res->solutions.push_back(js);
  
  res->success = res->solutions.empty()? false : true;
}


/**
 * @function getTransform // (-0.062, 0.0, 0.291);
 */
bool SimpleGetReachData::getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx)
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
   std::shared_ptr<SimpleGetReachData> rtt = std::make_shared<SimpleGetReachData>();

  if(!rtt->initialize())
    return 1;

  // Offer service
  rtt->setServices();

  rclcpp::spin(rtt);
  rclcpp::shutdown();
  return 0;    
}

