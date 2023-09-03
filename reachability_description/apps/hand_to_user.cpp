#include <reachability_description/reachability_description.h>
#include<reachability_msgs/srv/get_hand_to_user.hpp>
#include <algorithm>

/**
 * @function manip_comp
 */
bool dist_comp ( std::pair<int, double> _i,
                 std::pair<int, double> _j ) {
  return (_i.second < _j.second );
}

/**
 * @class HandToUser
 **/
class HandToUser 
{
 public:

 // Constructor
 HandToUser(const rclcpp::Node::SharedPtr &_nh) :
 nh_(_nh)
 {
    rd_.reset( new reachability_description::ReachabilityDescription(nh_));
    
 }

 // Initialize
 bool initialize(const std::string &_robot_name)
 {
    if(!rd_->initialize(_robot_name))
        return false;

    return true;
 }

// Load description
 bool loadDescription(const std::string &_chain_group)
 {
    chain_group_ = _chain_group;

    if(!rd_->loadDescription(_chain_group))
        return false;
    return true;
 }

// Offer service to get pose
bool setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = nh_->create_service<reachability_msgs::srv::GetHandToUser>("hand_to_user", std::bind(&HandToUser::handleSrv, this, _1, _2));

    pub_js_ = nh_->create_publisher<sensor_msgs::msg::JointState>("joint_state_command", 10);

    // Get the indices of the highest
    int num = rd_->getReachGraph(chain_group_)->getNumPoints();
    RCLCPP_INFO(nh_->get_logger(), "Start get services... Num points: %d ", num);

    for(int i = 0; i < num; ++i)
    {
      reachability_msgs::msg::ReachData rdi = rd_->getReachGraph(chain_group_)->getState(i);
      int min_samples = (int)( 0.5 * rd_->getReachGraph(chain_group_)->getNumVoxelSamples() );
      if( rdi.samples.size() > min_samples )
      {
        higher_indices_.push_back( i );
        higher_voxels_.push_back(rdi);
      }
    }

    return true;
}

bool aboveYZPlane(Eigen::Isometry3d _Tfx, double _x, double _y, double _z, double _thresh)
{
    // Get x axis
    Eigen::Vector3d normal_plane;
    normal_plane = _Tfx.linear().col(0);

    Eigen::Vector3d p(_x, _y, _z);
    
    Eigen::Vector3d orig_plane;
    orig_plane = _Tfx.translation();

    return ( (p - orig_plane).dot(normal_plane) - _thresh > 0);
}

// Handle service
void handleSrv(const std::shared_ptr<reachability_msgs::srv::GetHandToUser::Request> req,
               std::shared_ptr<reachability_msgs::srv::GetHandToUser::Response> res)
{
    RCLCPP_INFO(nh_->get_logger(), "Hand to user: start. Higher voxels size: %ld", higher_voxels_.size());

 // Calculate the distance through all the points higher to a point
 std::vector<std::pair<int, double> > distances;

Eigen::Isometry3d Tfx;
tf2::fromMsg(req->pose.pose, Tfx);

 for(int i = 0; i < higher_voxels_.size(); ++i)
 {
    // Only evaluate points that are in front
    double thresh = 0.1;
    double x, y, z;
    int xi, yi, zi;
    rd_->getReachGraph(chain_group_)->indexToVertex(higher_indices_[i], xi, yi, zi);
    rd_->getReachGraph(chain_group_)->vertexToWorld(xi, yi, zi, x, y, z);

    if(aboveYZPlane(Tfx, x, y, z, thresh))
        distances.push_back(std::make_pair(i, getL2Dist( req->pose, higher_indices_[i] ) ));
 }

    RCLCPP_INFO(nh_->get_logger(), "Poses over the plane: %ld ", distances.size());

  // Sort
  std::sort(distances.begin(), distances.end(), dist_comp);

  // For all the survivors, check if there is a direction close to (pose_goal - pose)
  RCLCPP_INFO(nh_->get_logger(), "Hand to user: end, distances size: %ld", distances.size());  
  
  sensor_msgs::msg::JointState js;
  js.name = rd_->getReachGraph(chain_group_)->getChainInfo().joint_names;
  RCLCPP_INFO(nh_->get_logger(), "hIGHER VOXELS size: %ld distances 0 first: %d", higher_voxels_.size(), distances[0].first);

  int index = getSample(higher_voxels_[distances[0].first].samples, Tfx);
  js.position = higher_voxels_[ distances[0].first ].samples[index].best_config;
  pub_js_->publish(js);

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
    rd_->getReachGraph(chain_group_)->indexToVertex(_index, xi, yi, zi);
    rd_->getReachGraph(chain_group_)->vertexToWorld(xi, yi, zi, x, y, z);

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
 
 rclcpp::Service<reachability_msgs::srv::GetHandToUser>::SharedPtr srv_;
 rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;

 std::vector<int> higher_indices_;
 std::vector<reachability_msgs::msg::ReachData> higher_voxels_;
 std::string chain_group_;
};


////////////////////////////////////

int main(int argc, char* argv[])
{
 rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hand_to_user");
    RCLCPP_INFO(rclcpp::get_logger("hand_to_user"), "Start node!--!");

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
  HandToUser htu(node);
  if(!htu.initialize(robot_name))
    return 1;

  // Actually generate the description
  htu.loadDescription(chain_group);

  // Offer service
  htu.setServices();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;    
}