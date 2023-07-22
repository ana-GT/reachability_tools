#include <reachability_description/reachability_description.h>
#include<reachability_msgs/srv/move_robot_to_task.hpp>
#include <reachability_msgs/srv/set_robot_pose.hpp>
#include <algorithm>

using namespace std::chrono_literals;

/**
 * @function manip_comp
 */
bool dist_comp ( std::pair<int, double> _i,
                 std::pair<int, double> _j ) {
  return (_i.second < _j.second );
}

/**
 * @class RobotToTask
 **/
class RobotToTask
{
 public:

 // Constructor
 RobotToTask(const rclcpp::Node::SharedPtr &_nh) :
 nh_(_nh)
 {
    rd_.reset( new reachability_description::ReachabilityDescription(nh_));
    
 }

 // Initialize
 bool initialize(const std::string &_robot_name)
 {
    if(!rd_->initialize(_robot_name))
        return false;

    above_ratio_ = 0.5;

    return true;
 }

// Load description
 bool loadDescription(const std::string &_chain_group)
 {
    if(!rd_->loadDescription(_chain_group))
        return false;
    return true;
 }

// Offer service to get pose
bool setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = nh_->create_service<reachability_msgs::srv::MoveRobotToTask>("robot_to_task", 
                std::bind(&RobotToTask::handleSrv, this, _1, _2));

    pub_js_ = nh_->create_publisher<sensor_msgs::msg::JointState>("joint_state_command", 10);

    client_move_base_ = nh_->create_client<reachability_msgs::srv::SetRobotPose>("set_robot_pose");

    // Get the indices of the highest
    int num = rd_->getReachGraph()->getNumPoints();

    for(int i = 0; i < num; ++i)
    {
      reachability_msgs::msg::ReachData rdi = rd_->getReachGraph()->getState(i);
      int min_samples = (int)( above_ratio_ * rd_->getReachGraph()->getNumVoxelSamples() );
      if( rdi.samples.size() > min_samples )
      {
        higher_indices_.push_back( i );
        higher_voxels_.push_back(rdi);
      }
    }

    return true;
}

bool withinZThresh(Eigen::Isometry3d _Tfx, double _x, double _y, double _z, double _thresh)
{
    return ( fabs(_z - _Tfx.translation()(2)) <= _thresh );
}

// Move base
void moveBase(const geometry_msgs::msg::PoseStamped &_pose)
{
  RCLCPP_WARN(nh_->get_logger(), "Move base received request to move to %f %f %f ", _pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z);
  auto request = std::make_shared<reachability_msgs::srv::SetRobotPose::Request>();
  request->pose = _pose;

  while (!client_move_base_->wait_for_service(1s)) {
      
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(nh_->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(nh_->get_logger(), "Now sending the request to move base for real");
  auto result = client_move_base_->async_send_request(request);
}

// Handle service
void handleSrv(const std::shared_ptr<reachability_msgs::srv::MoveRobotToTask::Request> req,
               std::shared_ptr<reachability_msgs::srv::MoveRobotToTask::Response> res)
{
    RCLCPP_INFO(nh_->get_logger(), "Received service to send robot to task!!!");

if(req->tcp_poses.empty())
   res->success = false;


// 1. Get all voxels that have a Z value in a threshold of this z
// Only use 1 for now
Eigen::Isometry3d Tfx;
tf2::fromMsg(req->tcp_poses[0].pose, Tfx);
double z_task = Tfx.translation()(2);

 std::vector<std::pair<int, double> > distances;
 for(int i = 0; i < higher_voxels_.size(); ++i)
 {
    // Only evaluate points that are in front
    double thresh = 0.05;
    double x, y, z;
    int xi, yi, zi;
    rd_->getReachGraph()->indexToVertex(higher_indices_[i], xi, yi, zi);
    rd_->getReachGraph()->vertexToWorld(xi, yi, zi, x, y, z);

    if(withinZThresh(Tfx, x, y, z, thresh))
        distances.push_back(std::make_pair(i, 0.0 )); // 0.0 getL2Dist( req->pose, higher_indices_[i] )
 }

    RCLCPP_INFO(nh_->get_logger(), "Poses in thresh: %d ", distances.size());


/*
 // Calculate the distance through all the points higher to a point


 for(int i = 0; i < higher_voxels_.size(); ++i)
 {
    // Only evaluate points that are in front
    double thresh = 0.1;
    double x, y, z;
    int xi, yi, zi;
    rd_->getReachGraph()->indexToVertex(higher_indices_[i], xi, yi, zi);
    rd_->getReachGraph()->vertexToWorld(xi, yi, zi, x, y, z);

    if(aboveYZPlane(Tfx, x, y, z, thresh))
        distances.push_back(std::make_pair(i, getL2Dist( req->pose, higher_indices_[i] ) ));
 }

    RCLCPP_INFO(rclcpp::get_logger("hand_to_user"), "Poses over the plane: %d ", distances.size());

  // Sort
  std::sort(distances.begin(), distances.end(), dist_comp);

  // For all the survivors, check if there is a direction close to (pose_goal - pose)
  RCLCPP_INFO(rclcpp::get_logger("hand_to_user"), "Hand to user: end, distances size: %d", distances.size());  
  
  sensor_msgs::msg::JointState js;
  js.name = rd_->getReachGraph()->getChainInfo().joint_names;
  RCLCPP_INFO(rclcpp::get_logger("hand_to_user"), "hIGHER VOXELS size: %d distances 0 first: %d", higher_voxels_.size(), distances[0].first);

  int index = getSample(higher_voxels_[distances[0].first].samples, Tfx);
  js.position = higher_voxels_[ distances[0].first ].samples[index].best_config;
  pub_js_->publish(js);
*/

//if(!req->tcp_poses.empty())
//    moveBase(req->tcp_poses[0]);


}

int getSample(const std::vector<reachability_msgs::msg::ReachSample> &_samples, const Eigen::Isometry3d &_Tfx)
{
    int min_index = -1;
    double min_ang = 1000;
    for(int i = 0; i < _samples.size(); ++i)
    {
        Eigen::Isometry3d Ts;
        tf2::fromMsg(_samples[i].pose, Ts);
        // Check that Ts.z is close to -Tfx.x
        Eigen::Vector3d hand_z; 
        hand_z = Ts.linear().col(2);
        Eigen::Vector3d p_x;
        p_x = -1*_Tfx.linear().col(0);

        Eigen::Quaterniond q;
        q.setFromTwoVectors(hand_z, p_x);
        Eigen::AngleAxisd aa(q);
        double ang = fabs(aa.angle());
        
        if( ang < min_ang)
        {
            min_ang = ang;
            min_index = i;
        }
    }

    return min_index;
}


// Helper
double getL2Dist( geometry_msgs::msg::PoseStamped _pose, int _index)
{
    // Get pose
    double x, y, z;
    int xi, yi, zi;
    double dx, dy, dz;
    rd_->getReachGraph()->indexToVertex(_index, xi, yi, zi);
    rd_->getReachGraph()->vertexToWorld(xi, yi, zi, x, y, z);

    // Dist
    dx = _pose.pose.position.x - x;
    dy = _pose.pose.position.y - y;
    dz = _pose.pose.position.z - z;

    double dist;
    dist = sqrt(dx*dx + dy*dy + dz*dz);
    return dist;
}

 protected:
 rclcpp::Node::SharedPtr nh_;
 std::shared_ptr<reachability_description::ReachabilityDescription> rd_;
 
 rclcpp::Service<reachability_msgs::srv::MoveRobotToTask>::SharedPtr srv_;
 rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
 // Client to move base
 rclcpp::Client<reachability_msgs::srv::SetRobotPose>::SharedPtr client_move_base_;


 std::vector<int> higher_indices_;
 std::vector<reachability_msgs::msg::ReachData> higher_voxels_;
 double above_ratio_;
};


////////////////////////////////////

int main(int argc, char* argv[])
{
 rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_robot_to_task");
    RCLCPP_INFO(node->get_logger(), "Start node");

  // Read parameters
  std::string chain_group;
  std::string robot_name;
  if(!node->has_parameter("chain_group_name"))
    node->declare_parameter("chain_group_name", std::string(""));
  node->get_parameter("chain_group_name", chain_group);

  if(!node->has_parameter("robot_name"))
    node->declare_parameter("robot_name", std::string(""));
  node->get_parameter("robot_name", robot_name);

  // Hand to user app
  RobotToTask rtt(node);
  if(!rtt.initialize(robot_name))
    return 1;

  // Actually generate the description
  rtt.loadDescription(chain_group);

  // Offer service
  rtt.setServices();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;    
}