/**
 * @file view_gmm_node.cpp
 */ 
#include <reachability_description/reachability_description.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <reachability_description/gmm_s2.h>

double modulo(const double &_val, const double &_factor)
{
  double new_val = _val;

  while(new_val > _factor)
    new_val = new_val - _factor;
  
  return new_val; 
}

/**
 * @class RiemannianTest
 */
class RiemannianTest : public rclcpp::Node {

  public:
  
  /** @function RiemannianTest */
  RiemannianTest(const std::string &_server_name) :
  rclcpp::Node("riemannian_test") {
  
  rclcpp::QoS qos_latch{1};
  qos_latch.transient_local();    
  pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gmm_node", qos_latch);
  
  timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RiemannianTest::timerCallback, this));
  
  }
  
  void timerCallback() {
  
    this->createSamples();
    //timer_->cancel();  
  }
  
  /** @function Create ball: Draw a big ball and small spheres on top to show samples  */
  void createSamples() {
  
    visualization_msgs::msg::MarkerArray ma;
    
    // Draw big sphere
    float r, g, b, a;
    double x, y, z, diam, small_diam;
    int id;
    
    r = 0.8f; g = 0.2f; b = 0.8f; a = 0.75f;
    id = 0;
    
    x = 0.0; y = 0.0; z = 1.0; diam = 0.50;
    visualization_msgs::msg::Marker m0 = drawSphere(x, y, z, diam, r, g, b, a, id);
    ma.markers.push_back(m0);

    
    // Generate samples
    std::vector<Eigen::Isometry3d> frames;
    int N = 80;
    double res = diam;
    this->createSphereSamplesVoxel(x, y, z, N, res, frames);
    
    // Draw little spheres 
    r = 0.2f; g = 0.8f; b = 0.8f; a = 0.5f;
    small_diam = 0.05;
    id++;
    
    int min_index = 0;
    int max_index = frames.size() - 1;
    for(int i =  min_index; i <= max_index; ++i)
    {
      auto p = frames[i].translation();
      if( p.x() > 0 || p.y() > 0 || p.z() < z)
        continue;
        
      visualization_msgs::msg::Marker mi = drawSphere(p.x(), p.y(), p.z(), small_diam, r, g, b, a, id);
      ma.markers.push_back(mi);
      id++;  
    }
 
    // Calculate the mean of these points
    Eigen::Vector3d u;

    std::vector<Eigen::Vector3d> points;
    for(int i = min_index; i <= max_index; ++i)
    {
       if(frames[i].translation().x() > 0 || frames[i].translation().y() > 0 || frames[i].translation().z() < z)
         continue;
       Eigen::Vector3d p;
       p = frames[i].translation() - Eigen::Vector3d(x,y,z);
       double norm = p.norm();
       p.normalize();
       points.push_back( p );
    }


    // If no converge, don't publish
    int iters = 20;
    double thresh = 0.0001;
    if(!s2::minimizeCentroid(points, u, iters, thresh))
    { RCLCPP_ERROR(this->get_logger(), "No minimizing, not publishing!");
      return;
    }
        
    // Publish this point
    visualization_msgs::msg::Marker mc = drawSphere(x + u.x()*diam/2.0, y+ u.y()*diam/2.0, z + u.z()*diam/2.0, small_diam, 1.0, 0.0, 0.0, 1.0, id);
    ma.markers.push_back(mc);
    
            
    // Publish them all
    pub_->publish(ma);
  }
  
 
  visualization_msgs::msg::Marker drawSphere(const double &_x, const double &_y, const double &_z, const double &_diam, const float &_r, const float &_g, const float &_b, const float &_a, int _id )
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();

    marker.ns = "";
    marker.id = _id;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = _x;
    marker.pose.position.y = _y;
    marker.pose.position.z = _z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = _diam;
    marker.scale.y = _diam;
    marker.scale.z = _diam;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = _r;
    marker.color.g = _g;
    marker.color.b = _b;
    marker.color.a = _a;

    marker.lifetime = rclcpp::Duration(0, 0);

    return marker;
  }
  
  void createSphereSamplesVoxel(const double &_x, 
                                const double &_y, 
                                const double &_z,
                                const int &_N,
                                const double &_resolution,
                                std::vector<Eigen::Isometry3d> &_frames) const
{
  _frames.clear();
  double dx, dy, dz;

  double theta_k, phi_k, h_k;
  double r = _resolution / 2.0;

  for(int k = 1; k <= _N; ++k)
  {

   h_k = -1.0 + 2.0*(double)(k-1)/(double)(_N-1);
   theta_k = acos(h_k);

   if(k == 1 || k == _N)
    phi_k = 0;
   else
    phi_k = modulo(phi_k + 3.6/sqrt(_N) *1/sqrt(1.0 - h_k*h_k), (2*M_PI));
  
   dx = r*sin(theta_k)*cos(phi_k);
   dy = r*sin(theta_k)*sin(phi_k);
   dz = r*cos(theta_k);

   // TCP's Z vector towards center (-dx, -dy, -dz)
   Eigen::Vector3d z_tcp; z_tcp << -dx, -dy, -dz;
   z_tcp.normalize();
   // Regular Z
   Eigen::Vector3d z_unit; z_unit << 0, 0, 1;
   Eigen::Quaterniond qz;
   qz.setFromTwoVectors(z_unit, z_tcp);

   Eigen::Isometry3d p; p.setIdentity();
   p.translation() = Eigen::Vector3d(_x + dx, _y + dy, _z + dz);
   p.linear() = qz.toRotationMatrix();

   _frames.push_back(p);

  }

}
  
  protected:    
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<RiemannianTest> rmt = std::make_shared<RiemannianTest>("view_gmm_node");

  rclcpp::spin(rmt);

  rclcpp::shutdown();
  return 0;    
}
