#include <reachability_description/reachability_description.h>
#include<reachability_msgs/srv/move_robot_to_task.hpp>
#include <reachability_msgs/srv/set_robot_pose.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <algorithm>

#include <reachability_description/reach_utilities.h>
#include <nlopt.h>

using namespace std::chrono_literals;

/**
 * @function manip_comp
 */
bool dist_comp ( std::pair<int, double> _i,
                 std::pair<int, double> _j ) {
  return (_i.second < _j.second );
}

bool samp_comp ( std::pair<int, int> _i,
                 std::pair<int, int> _j ) {
  return (_i.second < _j.second );
}

struct GS
{
  Eigen::Isometry3d G;
  Eigen::Isometry3d S;
};

double calc_error(Eigen::Isometry3d _S, Eigen::Isometry3d _G, std::vector<double> _x)
{

  double fx, fy, zx, zy;
  double G03, G13, G02, G12;
  double S03, S13, S02, S12;

  G03 = _G.matrix()(0,3);
  G13 = _G.matrix()(1,3);
  G02 = _G.matrix()(0,2);
  G12 = _G.matrix()(1,2);
  S03 = _S.matrix()(0,3);
  S13 = _S.matrix()(1,3);
  S02 = _S.matrix()(0,2);
  S12 = _S.matrix()(1,2);
  fx = G03 - _x[0] - cos(_x[2])*S03 - sin(_x[2])*S13;
  fy = G13 - _x[1] - sin(_x[2])*S03 - cos(_x[2])*S13;
  zx = G02 - cos(_x[2])*S02 - sin(_x[2])*S12;
  zy = G12 - sin(_x[2])*S02 - cos(_x[2])*S12;


  return (fx*fx + fy*fy + zx*zx  + zy*zy);
}

double min_func(const std::vector<double> &x, std::vector<double>& grad, void* data)
{
  // x
  // tx = x[0] ty = x[1] theta = x[2]
  GS* c = (GS*) data;

  Eigen::Isometry3d S, G; // sample and goal
  S = c->S;
  G = c->G;
  std::vector<double> vals(x);

  double jump = FLT_EPSILON;
  double res = calc_error(S, G, x);
  
  if (!grad.empty())
  {
    double v1;
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;
      v1 = calc_error(S, G, vals);

      vals[i] = original;
      grad[i] = (v1 - res) / (2 * jump);
    }
  }

  return res;
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

    above_ratio_ = 0.35;
    below_z_comp_ = 0.15; // (nx, ny, nz) vs (mx, my, mz), this factor is diff between nz and mx, at most it can be 2

    opt_ = nlopt::opt(nlopt::LD_SLSQP, 3); // nlopt::LD_SLSQP
    opt_.set_xtol_abs(0.01);
    opt_.set_maxtime(0.005);
    
    std::vector<double> x_lower_bounds = {-10.0, -10.0, -M_PI};
    std::vector<double> x_upper_bounds = {10.0, 10.0, M_PI};

    opt_.set_lower_bounds(x_lower_bounds);
    opt_.set_upper_bounds(x_upper_bounds);


    return true;
 }

// Load description
 bool loadDescription(const std::string &_chain_group)
 {
    chain_group_ = _chain_group;
    if(!rd_->loadDescription(_chain_group))
        return false;
  
    rd_->addKinematicSolvers(_chain_group);

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
    int num = rd_->getReachGraph(chain_group_)->getNumPoints();

    for(int i = 0; i < num; ++i)
    {
      reachability_msgs::msg::ReachData rdi = rd_->getReachGraph(chain_group_)->getState(i);
      int min_samples = (int)( above_ratio_ * rd_->getReachGraph(chain_group_)->getNumVoxelSamples() );
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
  RCLCPP_WARN(nh_->get_logger(), "Move base received request to move to %f %f %f ", 
              _pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z);
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

 std::vector<std::pair<int, int> > distances;
 for(int i = 0; i < higher_voxels_.size(); ++i)
 {
    // Only evaluate points that are in front
    double thresh = 0.05;
    double x, y, z;
    int xi, yi, zi;
    rd_->getReachGraph(chain_group_)->indexToVertex(higher_indices_[i], xi, yi, zi);
    rd_->getReachGraph(chain_group_)->vertexToWorld(xi, yi, zi, x, y, z);

    if(withinZThresh(Tfx, x, y, z, thresh))
        distances.push_back(std::make_pair(i, higher_voxels_[i].samples.size())); // 0.0 getL2Dist( req->pose, higher_indices_[i] )
 }

  RCLCPP_INFO(nh_->get_logger(), "Poses in thresh: %lu ", distances.size());

  // Sort
  std::sort(distances.begin(), distances.end(), samp_comp);

  // Go through all of them, and check which sample is closer
  for(int i = 0; i < distances.size(); ++i)
  {
    std::vector<geometry_msgs::msg::PoseStamped> sols;
    std::vector<sensor_msgs::msg::JointState> jjs;
    if(calculateSolInVoxel(higher_voxels_[ distances[i].first ], Tfx, sols, jjs))
    {
      moveBase(sols[0]);

      pub_js_->publish(jjs[0]);
      res->success = true;
      return;

    }
  }

}

/**
 * @function calculateSolInVoxel 
 */
bool calculateSolInVoxel(reachability_msgs::msg::ReachData _rdi, 
        Eigen::Isometry3d _Tf_goal, 
        std::vector<geometry_msgs::msg::PoseStamped> &_sols, 
        std::vector<sensor_msgs::msg::JointState> &_jjs )
{
  Eigen::Isometry3d G, S; // sample
  Eigen::Vector3d z_g;

  G = _Tf_goal; 
  z_g = G.linear().col(2);
  
  int num_sols = 0;
  double min_dist = 1000;
  double max_dist = 0;
  for(int i = 0; i < _rdi.samples.size(); ++i)
  {
    Eigen::Vector3d z_s;
    tf2::fromMsg( _rdi.samples[i].pose, S);
    z_s = S.linear().col(2); 

    double dist = fabs(z_s(2) - z_g(2)); 
    if( dist < below_z_comp_ )
    {
      num_sols++;
     
      double minf;
      std::vector<double> x = {G.translation()(0) - 1.0, G.translation()(1) - 1.0, 0};
      GS gs_struct;
      gs_struct.G = G;
      gs_struct.S = S;
      opt_.set_min_objective(min_func, &gs_struct);

      int res;
      try
      {
        res = opt_.optimize(x, minf);
      }
      catch (...)
     {
     }
      if(res > 0)
      {
        Eigen::Isometry3d Tbase, Texpected;
        Tbase = getPlanarTransform(x[0], x[1], x[2]);
        
        KDL::JntArray q_init, q_out;
        q_init = vectorToJntArray(_rdi.samples[i].best_config);

        KDL::Twist bounds = KDL::Twist::Zero();
        Texpected = Tbase.inverse()*G;
        KDL::Frame p_in;
        tf2::transformEigenToKDL(Texpected, p_in);
        int ik_sol = rd_->getIKSolver(chain_group_)->CartToJnt(q_init, p_in, q_out, bounds);
        if(ik_sol > 0)
        {
          geometry_msgs::msg::PoseStamped pi;
          pi.pose = tf2::toMsg(Tbase);
          pi.header.frame_id = "world";
          _sols.push_back(pi);

          sensor_msgs::msg::JointState js;
          js.name = rd_->getReachGraph(chain_group_)->getChainInfo().joint_names;
          js.position.resize(js.name.size());
          for(int k = 0; k < js.name.size(); ++k)
          js.position[k] = q_out(k);
          _jjs.push_back(js);

        }
      }
    }

  }
  return (_sols.size() > 0);
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
 double below_z_comp_ = 0.1;
 std::string chain_group_;

 nlopt::opt opt_;

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