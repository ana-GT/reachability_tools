#include <reachability_description/reachability_description.h>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <algorithm>

#include <reachability_description/reach_utilities.h>
#include <reachability_description/reach_graph_aggregated.h>
#include <nlopt.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include<reachability_msgs/srv/generate_reach_poses.hpp>

#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>
#include <tf2_kdl/tf2_kdl.h>


using namespace std::chrono_literals;

/**
 * @class RobotToTask
 **/
class GenerateRobotReachEEPoses
{
 public:

 // Constructor
 GenerateRobotReachEEPoses(const rclcpp::Node::SharedPtr &_nh) :
 node_(_nh)
 {
    rd_.reset( new reachability_description::ReachabilityDescription(node_));

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
  
    if(!rd_->addKinematicSolvers(_chain_group))
    	return false;

    // Load rga
    //rga_.fillFromGraph(rd_, _chain_group);

    return true;
 }

// Offer service to get pose
bool setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = node_->create_service<reachability_msgs::srv::GenerateReachPoses>("generate_reach_poses", 
                std::bind(&GenerateRobotReachEEPoses::handleSrv, this, _1, _2));

    //pub_floor_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("floor_cloud", 10);
    //pub_floor_points_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("floor_points", 10);

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
void handleSrv(const std::shared_ptr<reachability_msgs::srv::GenerateReachPoses::Request> req,
               std::shared_ptr<reachability_msgs::srv::GenerateReachPoses::Response> res)
{
  res->success = false;

  if(req->bbox.size.x == 0 || req->bbox.size.y == 0 || req->bbox.size.z == 0)
   return;

  // 1. Get closest voxel to BB's center
  geometry_msgs::msg::Pose bbox = req->bbox.center;
  double xi, yi, zi;
  int center_index = rd_->getReachGraph(chain_group_)->worldToIndex(bbox.position.x, bbox.position.y, bbox.position.z);
  
  // Min corner, max corner
  Eigen::Vector3d bb_dim; bb_dim << 0.5*req->bbox.size.x, 0.5*req->bbox.size.y, 0.5*req->bbox.size.z; 
  Eigen::Vector3d bb_min; bb_min << bbox.position.x - bb_dim(0), bbox.position.y - bb_dim(1), bbox.position.z - bb_dim(2);
  Eigen::Vector3d bb_max; bb_max << bbox.position.x + bb_dim(0), bbox.position.y + bb_dim(1), bbox.position.z + bb_dim(2);
  
  int xli, yli, zli, xui, yui, zui;
  rd_->getReachGraph(chain_group_)->worldToVertex(bb_min(0), bb_min(1), bb_min(2), xli, yli, zli);
  rd_->getReachGraph(chain_group_)->worldToVertex(bb_max(0), bb_max(1), bb_max(2), xui, yui, zui);

  RCLCPP_INFO(node_->get_logger(), "*** Bounding box: min: %f %f %f, max: %f %f %f", bb_min(0), bb_min(1), bb_min(2), bb_max(0), bb_max(1), bb_max(2));
  RCLCPP_INFO(node_->get_logger(), "*** Bounding box Indices: min: %d %d %d, max: %d %d %d", xli, yli, zli, xui, yui, zui);

  std::vector<reachability_msgs::msg::ReachData> rd_nn;
  int num = 0;
  int valid = 0;
  for(int xi = xli; xi <= xui; ++xi)
    for(int yi = yli; yi <= yui; ++yi)
       for(int zi = zli; zi <= zui; ++zi)
       {
          reachability_msgs::msg::ReachData rdi;
          rdi = rd_->getReachGraph(chain_group_)->getState(xi, yi, zi);
          if(rdi.state == reachability_msgs::msg::ReachData::FILLED)
          {
            num++;
            for(int k = 0; k < rdi.samples.size(); ++k)
            {
               // Check if there are samples that point in the desired direction
               Eigen::Quaterniond q;
               q.x() = rdi.samples[k].pose.orientation.x;
               q.y() = rdi.samples[k].pose.orientation.y;
               q.z() = rdi.samples[k].pose.orientation.z;
               q.w() = rdi.samples[k].pose.orientation.w;
               Eigen::Matrix3d m; m = q.matrix();
               Eigen::Vector3d z_dir;
               z_dir = m.col(2);
               
               Eigen::Vector3d dir_to_bbox, dir_to_bbox_norm;
               dir_to_bbox << bbox.position.x - rdi.samples[k].pose.position.x, bbox.position.y - rdi.samples[k].pose.position.y, bbox.position.z - rdi.samples[k].pose.position.z;
               dir_to_bbox_norm = dir_to_bbox.normalized();
               double ang;
               ang = fabs(acos(z_dir.dot(dir_to_bbox_norm)));
               //RCLCPP_INFO(node_->get_logger(), "** Angle[%d/%d]: %f zdir: %f %f %f Dir to box: %f %f %f", k, rdi.samples.size(), ang*180.0/M_PI, z_dir(0), z_dir(1), z_dir(2), dir_to_bbox(0), dir_to_bbox(1), dir_to_bbox(2));
               if(ang < 45.0*M_PI/180.0)
               {
                  rd_nn.push_back(rdi);
                  valid++;
                  break;
               }
               
            } // for k

          } // if filled
            
       } // for zi
  
  RCLCPP_INFO(node_->get_logger(), "*-*-*-* Voxels in BB: %d, voxels used for NN: %d", num, valid);
  
  reachability_msgs::msg::ReachData rd_center = rd_->getReachGraph(chain_group_)->getState(center_index);
  RCLCPP_INFO(node_->get_logger(), "Voxel selected state: %d metric(%s): %f", rd_center.state, rd_center.metrics[0].name.c_str(),  rd_center.metrics[0].value);
  RCLCPP_INFO(node_->get_logger(), "Within the bounding box: %d", rd_nn.size());

   //1. Get centers
   /*cv::Mat points(rd_center.samples.size(), 1, CV_32FC3);
  
  // Check that these reachability poses are pointing to
  std::vector<geometry_msgs::msg::PoseStamped> ee_poses;
  for(int i = 0; i < rd_center.samples.size(); ++i)
  {
    Eigen::Vector3d p;
    p << rd_center.samples[i].pose.position.x, rd_center.samples[i].pose.position.y,  rd_center.samples[i].pose.position.z ;
    RCLCPP_INFO(node_->get_logger(), "Point to show: %f %f %f **************", p(0), p(1), p(2));
      // Get center
      geometry_msgs::msg::PoseStamped pi;
      pi.pose.position.x = p(0);
      pi.pose.position.y = p(1);
      pi.pose.position.z = p(2);
      // Z: direction from this origin to the center of BB
      pi.pose.orientation = rd_center.samples[i].pose.orientation;
      pi.header.frame_id = rd_->getReachGraph(chain_group_)->getChainInfo().root_link;
            
      //int res = rd_->getIKSolver(_chain_group)->CartToJnt(q_init, p_in, q_out, bounds);
      
      // Get pose
      ee_poses.push_back(pi);
    
  }*/
  
  
   
   cv::Mat labels;
   std::vector<cv::Point3f> centers;
   cv::Mat points(rd_nn.size(), 1, CV_32FC3);

   // 2. Fill points
   for(int i = 0; i < rd_nn.size(); ++i)
   {
    Eigen::Vector3d p;
    p << rd_nn[i].samples[0].pose.position.x, rd_nn[i].samples[0].pose.position.y,  rd_nn[i].samples[0].pose.position.z ;
    points.at<cv::Vec3f>(i) = cv::Vec3f( (float)p(0), (float)p(1), (float)p(2));
   }     
  
   cv::kmeans(points, req->num_poses, labels,
   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
   3, cv::KMEANS_PP_CENTERS, centers);
   std::vector<geometry_msgs::msg::PoseStamped> ee_poses;
   std::vector<sensor_msgs::msg::JointState> jss;
   for(int i = 0; i < centers.size(); ++i)
   {
      // Get center
      geometry_msgs::msg::PoseStamped pi;
      pi.pose.position.x = centers[i].x;
      pi.pose.position.y = centers[i].y;
      pi.pose.position.z = centers[i].z;
      // Z: direction from this origin to the center of BB
      Eigen::Vector3d z_dir; z_dir << bbox.position.x - centers[i].x, bbox.position.y - centers[i].y, bbox.position.z - centers[i].z;
      Eigen::Quaterniond q;
      q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), z_dir);
      pi.pose.orientation.x = q.x(); 
      pi.pose.orientation.y = q.y();
      pi.pose.orientation.z = q.z();
      pi.pose.orientation.w = q.w(); 

      pi.header.frame_id = rd_->getReachGraph(chain_group_)->getChainInfo().root_link;
      
      KDL::JntArray q_init, q_out;
        
      // TODO: Do this better!  
      q_init = vectorToJntArray(rd_center.samples[0].best_config);

      KDL::Twist bounds = KDL::Twist::Zero();
      KDL::Frame p_in;
      //tf2::transformEigenToKDL(Texpected, p_in);
      tf2::fromMsg(pi.pose, p_in);
      if(rd_->getIKSolver(chain_group_)->CartToJnt(q_init, p_in, q_out, bounds) > 0)
      {
        // Get pose
        sensor_msgs::msg::JointState js;
        js.name = rd_->getReachGraph(chain_group_)->getChainInfo().joint_names;
        js.position.resize(js.name.size());
        for(int k = 0; k < js.name.size(); ++k)
          js.position[k] = q_out(k);
        
        ee_poses.push_back(pi);
        jss.push_back(js); 
      }  
   }
  
  res->ee_poses = ee_poses;
  res->joint_states = jss;
  
  //int num = rd_->getReachGraph(chain_group_)->getNumPoints();
  
  // Read goal poses
  Eigen::Isometry3d Tgs;
  tf2::fromMsg(req->bbox.center, Tgs);      

  clock_t ts, tf; double dt;

  //std::vector<InvData> candidates;
  //std::vector<InvData> best_candidates;
  ts = clock();
  //rga_.getCandidates(Tgs, candidates, best_candidates);
  tf = clock();
  dt = (double)(tf-ts)/(double)CLOCKS_PER_SEC;
  //RCLCPP_INFO(node_->get_logger(), "DT: %f to calculate for points. Points: %ld. Best points: %ld  \n", 
  //            dt, candidates.size(), best_candidates.size());

  //publishCloud(points, best_points);
  //publishPoints(candidates, best_candidates);
/*
  if(best_candidates.empty())
  {
    res->success = false;
    return;
  }
*/
  // Get solutions
  /*
  std::vector<PlaceSol> sols;
  sols = rga_.getSolutions(Tgs, best_candidates, req->num_poses);
  RCLCPP_INFO(node_->get_logger(), "Got %lu solutions to return", sols.size());
  solutionsToMsg(sols, res->solutions);
  res->success = !sols.empty();
    RCLCPP_INFO(node_->get_logger(), "Returning");*/
  return;
}

/**
 * @function solutionsToMsg 
 */
 /*
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
    //RCLCPP_INFO(node_->get_logger(), "Solution %d with %d qs", i, _solutions[i].q.size());

    for(int j = 0; j < _solutions[i].q.size(); ++j)
    {
       sensor_msgs::msg::JointState js;
       js.name = rd_->getReachGraph(chain_group_)->getChainInfo().joint_names;
       js.position.resize(js.name.size());
       for(int k = 0; k < js.name.size(); ++k)
         js.position[k] = _solutions[i].q[j](k);

       prs.chain_sols.push_back(js);

    }

    // Store
    _msg.push_back(prs);
  }

}*/

/**
 * @function publishPoints 
 */
 /*
void publishPoints(const std::vector<InvData> &_points, 
                   const std::vector<InvData> &_best_points)
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
  for(auto pi : _points)
  {
    geometry_msgs::msg::Point p;
    p.x = pi.place.Twb.translation()(0);
    p.y = pi.place.Twb.translation()(1);
    p.z = pi.place.Twb.translation()(2);
    msg_reg.points.push_back(p);

    std_msgs::msg::ColorRGBA col;
    col.r = 0.0; col.g = 1.0; col.b = 1.0; col.a = 1.0;

    msg_reg.colors.push_back(col);

  } // end for

  msg.markers.push_back(msg_reg);


  // Best points
  int count = 1;
  for(auto pi : _best_points)
  {
  visualization_msgs::msg::Marker msg_best;
  msg_best.type = visualization_msgs::msg::Marker::ARROW;
  msg_best.action = visualization_msgs::msg::Marker::ADD;
  msg_best.header.frame_id = "world";
  msg_best.scale.x = 0.03; msg_best.scale.y = 0.005; msg_best.scale.z = 0.005;
  msg_best.color.r = 1.0; msg_best.color.g = 0; msg_best.color.b = 1.0; msg_best.color.a = 1.0;

  msg_best.pose = tf2::toMsg(pi.place.Twb);   
  msg_best.id = count;
  msg.markers.push_back(msg_best);
  count++;
  } // end best for


  // Publish
  pub_floor_points_->publish(msg);  
}*/


/**
 * @function publishCloud
 * @brief Publish possible solutions as a PointCloud2 message 
 */
 /*
void publishCloud(const std::vector<Eigen::Isometry3d> &_points, 
                  const int &_best_points)
{  
  printf("Points total: %ld, best points: %d \n", _points.size(), _best_points);
  sensor_msgs::msg::PointCloud2 pcl_msg;
    
  //Modifier to describe what the fields are.
  sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(_points.size());

  //Msg header
  pcl_msg.header = std_msgs::msg::Header();
  pcl_msg.header.stamp = node_->get_clock()->now();
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
*/

 protected:
 rclcpp::Node::SharedPtr node_;
 std::shared_ptr<reachability_description::ReachabilityDescription> rd_;
 //ReachGraphAggregated rga_;

 rclcpp::Service<reachability_msgs::srv::GenerateReachPoses>::SharedPtr srv_;
 //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_floor_cloud_;
 //rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_floor_points_;



 std::vector<int> higher_indices_;
 std::vector<reachability_msgs::msg::ReachData> higher_voxels_;
 double above_ratio_ = 0.2;
 double below_z_comp_ = 0.1;
 std::string chain_group_;

 nlopt::opt opt_;

};


////////////////////////////////////

int main(int argc, char* argv[])
{
 rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("generate_robot_reach_ee_poses");
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
  GenerateRobotReachEEPoses grre(node);
  if(!grre.initialize(robot_name))
    return 1;

  // Actually generate the description
  grre.loadDescription(chain_group);

  // Offer service
  grre.setServices();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;    
}
