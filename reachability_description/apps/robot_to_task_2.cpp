#include <reachability_description/reachability_description.h>
#include<reachability_msgs/srv/move_robot_to_task.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <algorithm>

#include <reachability_description/reach_utilities.h>
#include <reachability_description/reach_graph_aggregated.h>
#include <nlopt.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


using namespace std::chrono_literals;

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

    return true;
 }

// Load description
 bool loadDescription(const std::string &_chain_group)
 {
    chain_group_ = _chain_group;
    if(!rd_->loadDescription(_chain_group))
        return false;
  
    rd_->addKinematicSolvers(_chain_group);

    // Load rga
    rga_.fillFromGraph(rd_, _chain_group);

    return true;
 }

// Offer service to get pose
bool setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = nh_->create_service<reachability_msgs::srv::MoveRobotToTask>("robot_to_task", 
                std::bind(&RobotToTask::handleSrv, this, _1, _2));

    pub_floor_cloud_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("floor_cloud", 10);
    pub_floor_points_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("floor_points", 10);

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

// Handle service
void handleSrv(const std::shared_ptr<reachability_msgs::srv::MoveRobotToTask::Request> req,
               std::shared_ptr<reachability_msgs::srv::MoveRobotToTask::Response> res)
{

  if(req->tcp_poses.empty())
  {
   res->success = false;
   return;
  }

  int num = rd_->getReachGraph(chain_group_)->getNumPoints();

  Eigen::Isometry3d Tg;
  tf2::fromMsg(req->tcp_poses[0].pose, Tg);      

  clock_t ts, tf; double dt;
  int within_range = 0;
  bool within_range_sample;
  int within_range_sample_acc = 0;

  std::vector<PlaceSol> candidates;
  std::vector<Sample> samples;
  int best_candidates;
  ts = clock();
  rga_.getCandidates(Tg, candidates, samples, best_candidates);
  tf = clock();
  dt = (double)(tf-ts)/(double)CLOCKS_PER_SEC;
  RCLCPP_INFO(nh_->get_logger(), "DT: %f to calculate for points. Points: %d. Best points: %d  \n", dt, candidates.size(), best_candidates);

  //publishCloud(points, best_points);
  publishPoints(candidates, best_candidates);

  // Get solutions
  std::vector<PlaceSol> sols;

  std::vector<PlaceSol> best_cd; best_cd.insert(best_cd.end(), candidates.end() - best_candidates, candidates.end());
  std::vector<Sample> best_samples; best_samples.insert(best_samples.end(), samples.end() - best_candidates, samples.end() );

  sols = rga_.getSolutions(Tg, best_cd, best_samples, req->num_robot_placements);
  solutionsToMsg(sols, res->solutions);
  res->success = !sols.empty();
  return;
}

/**
 * @function solutionsToMsg 
 */
void solutionsToMsg(const std::vector<PlaceSol> &_solutions,
                    std::vector<reachability_msgs::msg::PlaceRobotSolution> &_msg)
{
  // Clean up
  _msg.clear();

  // Fill message
  for(int i = 0; i < _solutions.size(); ++i)
  {
    reachability_msgs::msg::PlaceRobotSolution prs;
    
    prs.base_pose.pose = tf2::toMsg(_solutions[i].Twb);
    prs.base_pose.header.frame_id = "world";

    sensor_msgs::msg::JointState js;
    js.name = rd_->getReachGraph(chain_group_)->getChainInfo().joint_names;
    js.position.resize(js.name.size());
    for(int k = 0; k < js.name.size(); ++k)
      js.position[k] = _solutions[i].q(k);

    prs.chain_config = js;

    // Store
    _msg.push_back(prs);
  }

}

/**
 * @function publishPoints 
 */
void publishPoints(const std::vector<PlaceSol> &_points, 
                   const int &_best_points)
{  
  // First reset
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  msg.markers.push_back(del);

  pub_floor_points_->publish(msg);

  // Clear
  msg.markers.clear();

  // Points regular
  visualization_msgs::msg::Marker msg_reg;
  msg_reg.type = visualization_msgs::msg::Marker::POINTS;
  msg_reg.action = visualization_msgs::msg::Marker::ADD;
  msg_reg.header.frame_id = "world";
  msg_reg.scale.x = 0.005; msg_reg.scale.y = 0.005;
  msg_reg.pose.orientation.w = 1.0;
  msg_reg.id = 0;
  for(int i = 0; i < _points.size() - _best_points; ++i)
  {
    geometry_msgs::msg::Point p;
    p.x = _points[i].Twb.translation()(0);
    p.y = _points[i].Twb.translation()(1);
    p.z = _points[i].Twb.translation()(2);
    msg_reg.points.push_back(p);

    std_msgs::msg::ColorRGBA col;
    col.r = 0.0; col.g = 1.0; col.b = 1.0; col.a = 1.0;

    msg_reg.colors.push_back(col);

  } // end for

  msg.markers.push_back(msg_reg);


  // Best points
  int count = 1;
  for(int i = _points.size() - _best_points; i < _points.size(); ++i)
  {
  visualization_msgs::msg::Marker msg_best;
  msg_best.type = visualization_msgs::msg::Marker::ARROW;
  msg_best.action = visualization_msgs::msg::Marker::ADD;
  msg_best.header.frame_id = "world";
  msg_best.scale.x = 0.03; msg_best.scale.y = 0.005; msg_best.scale.z = 0.005;
  msg_best.color.r = 1.0; msg_best.color.g = 0; msg_best.color.b = 1.0; msg_best.color.a = 1.0;

  msg_best.pose = tf2::toMsg(_points[i].Twb);   
  msg_best.id = count;
  msg.markers.push_back(msg_best);
  count++;
  } // end best for


  // Publish
  pub_floor_points_->publish(msg);  
}


/**
 * @function publishCloud
 * @brief Publish possible solutions as a PointCloud2 message 
 */
void publishCloud(const std::vector<Eigen::Isometry3d> &_points, 
                  const int &_best_points)
{  
  printf("Points total: %d, best points: %d \n", _points.size(), _best_points);
  sensor_msgs::msg::PointCloud2 pcl_msg;
    
  //Modifier to describe what the fields are.
  sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(_points.size());

  //Msg header
  pcl_msg.header = std_msgs::msg::Header();
  pcl_msg.header.stamp = nh_->get_clock()->now();
  pcl_msg.header.frame_id = "world";

  pcl_msg.height = _points.size();
  pcl_msg.width = 1;
  pcl_msg.is_dense = false;

  int regular_points = _points.size() - _best_points;

  //Iterators for PointCloud msg
  sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pcl_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pcl_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pcl_msg, "b");

  //iterate over the message and populate the fields. 
  for(int i = 0; i < _points.size(); ++i)     
  {
    *iterX = _points[i].translation()(0);
    *iterY = _points[i].translation()(1);
    *iterZ = _points[i].translation()(2);

    if(i < regular_points)
    { *iter_r = 0; *iter_g = 255; *iter_b = 255; }
    else
    { *iter_r = 255; *iter_g = 0; *iter_b = 255; }

    // Increment the iterators
    ++iterX;
    ++iterY;
    ++iterZ;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }

  // Publish
  pub_floor_cloud_->publish(pcl_msg);

}


 protected:
 rclcpp::Node::SharedPtr nh_;
 std::shared_ptr<reachability_description::ReachabilityDescription> rd_;
 ReachGraphAggregated rga_;

 rclcpp::Service<reachability_msgs::srv::MoveRobotToTask>::SharedPtr srv_;
 rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_floor_cloud_;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_floor_points_;



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