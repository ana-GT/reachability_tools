/**
 * @file reach_graph_reuleaux.cpp
 */
#include <reach_plugin_reuleaux/reach_graph_reuleaux.h>
#include <reach_plugin_reuleaux/quaternion_discretization.h>

using namespace reachability_description;

double modulo(const double &_val, const double &_factor)
{
  double new_val = _val;

  while(new_val > _factor)
    new_val = new_val - _factor;
  
  return new_val; 
}

ReachGraphReuleaux::ReachGraphReuleaux()
{}

ReachGraphReuleaux::~ReachGraphReuleaux()
{}

void ReachGraphReuleaux::generateSamples(const int &_xi, const int &_yi, const int &_zi, 
                                         std::vector<Eigen::Isometry3d> &_frames)
{
  this->createTesseractSamples(_xi, _yi, _zi, _frames);
  //this->createSphereSamplesVoxel(_xi, _yi, _zi, _frames);
}

/**
 * @brief createSphereSamplesVoxel
 */
void ReachGraphReuleaux::createSphereSamplesVoxel(const int &_xi, 
                                const int &_yi, 
                                const int &_zi,
                                std::vector<Eigen::Isometry3d> &_frames) const
{
  _frames.clear();

  double x, y, z;
  double dx, dy, dz;

  vertexToWorld(_xi, _yi, _zi, x, y, z);

  int N = params_.num_voxel_samples;

  double theta_k, phi_k, h_k;
  double r = params_.resolution / 2.0;

  for(int k = 1; k <= N; ++k)
  {

   h_k = -1.0 + 2.0*(double)(k-1)/(double)(N-1);
   theta_k = acos(h_k);

   if(k == 1 || k == N)
    phi_k = 0;
   else
    phi_k = modulo(phi_k + 3.6/sqrt(N) *1/sqrt(1.0 - h_k*h_k), (2*M_PI));
  
   dx = r*sin(theta_k)*cos(phi_k);
   dy = r*sin(theta_k)*sin(phi_k);
   dz = r*cos(theta_k);

   // TCP's Z vector towards center (-dx, -dy, -dz)
   Eigen::Vector3d z_tcp; z_tcp << -dx, -dy, -dz;
   // Regular Z
   Eigen::Vector3d z_unit; z_unit << 0, 0, 1;
   Eigen::Quaterniond qz;
   qz.setFromTwoVectors(z_unit, z_tcp);

   Eigen::Isometry3d p; p.setIdentity();
   p.translation() = Eigen::Vector3d(x + dx,y + dy,z + dz);
   p.linear() = qz.toRotationMatrix();

   _frames.push_back(p);

  }

}

/**
 * @brief createTesseractSamples
 */
void ReachGraphReuleaux::createTesseractSamples(const int &_xi, 
                                                const int &_yi, 
                                                const int &_zi,
                                                std::vector<Eigen::Isometry3d> &_frames) const
{
  _frames.clear();

  double x, y, z;
  double dx, dy, dz;

  vertexToWorld(_xi, _yi, _zi, x, y, z);

  int N = params_.num_voxel_samples;

  int n = 1;
  // 1. Create the quaternion samples
  if(N == 40)
    n = 1;
  else if(N == 272)
    n = 2;
  else
    printf("Num samples is not either 40 or 272!!!! \n");
  
  std::vector<Eigen::Quaterniond> qs;
  TesseractDiscretization td;
  qs = td.generateQuaternions(n);
  
  for(int k = 0; k < N; ++k)
  {
   Eigen::Isometry3d p; p.setIdentity();
   p.translation() = Eigen::Vector3d(x, y, z);
   p.linear() = qs[k].toRotationMatrix();

   _frames.push_back(p);
  }

}

/**
 * @function getPCD
 */
sensor_msgs::msg::PointCloud2 ReachGraphReuleaux::debugSamples(int _xi, int _yi, int _zi)
{
  sensor_msgs::msg::PointCloud2 cloud;

  int N = params_.num_voxel_samples;
  cloud.header.frame_id = chain_info_.root_link;
  cloud.width = N;
  cloud.height = 1;
  cloud.is_dense = false;
 
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(N);

  // iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud, "b");

  // Enter vertices in the graph
  std::vector<Eigen::Isometry3d> frames;
  createSphereSamplesVoxel(_xi, _yi, _zi, frames);
  for( int i = 0; i < N; ++i ) { 
      float xd, yd, zd;
      xd = (float)frames[i].translation()(0);
      yd = (float)frames[i].translation()(1);
      zd = (float)frames[i].translation()(2);

      *out_x = xd;
      *out_y = yd;
      *out_z = zd;
      *out_r = 250; 
      *out_g = 250; 
      *out_b = 0; 

      ++out_x; ++out_y; ++out_z;
      ++out_r; ++out_g; ++out_b;
  }
 
  return cloud;
}


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(reachability_description::ReachGraphReuleaux, reachability_description::ReachGraph)

