/**
 * @file Graph.cpp
 * @author A. Huaman Q.
 * @date 2012 / 08/ 16
 */
#include <reachability_description/reach_data.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace reachability_description;

double modulo(const double &_val, const double &_factor)
{
  double new_val = _val;

  while(new_val > _factor)
    new_val = new_val - _factor;
  
  return new_val; 
}


/**
 * @function Graph
 */
ReachGraph::ReachGraph( const ChainInfo &_chain_info, double _min_x, double _min_y, double _min_z,
	      double _max_x, double _max_y, double _max_z,
	      double _resolution, 
	      ReachData _default ) :
min_x_(_min_x),
min_y_(_min_y),
min_z_(_min_z),
max_x_(_max_x),
max_y_(_max_y),
max_z_(_max_z),
res_(_resolution)
{
 chain_info_ = _chain_info;

  num_x_ = ( ( max_x_ - min_x_ ) / res_ );
  num_y_ = ( ( max_y_ - min_y_ ) / res_ );
  num_z_ = ( ( max_z_ - min_z_ ) / res_ );

  step_yz_ = num_y_*num_z_;
  step_z_ = num_z_;

  num_points_ = num_x_*num_y_*num_z_;
  points_ = new ReachData[num_points_];

  std::fill( points_, points_ + num_points_, _default );

}

/**
 * @function ~Graph
 */
ReachGraph::~ReachGraph() {

  if( points_ != NULL ) {
    delete [] points_;
  }
}

/**
 * @function WorldToVertex
 */
void ReachGraph::worldToVertex( const double &_x, const double &_y, const double &_z,
			   int &_xi, int &_yi, int &_zi ) {

  // mOX <= _wx <= mMaxX && mOX <= _wx <= mMaxY && mOY <= _wz <= mMaxZ
  // floor effect expected: Round to minimum integer
  _xi = (int) ( ( _x - min_x_ ) / res_ );
  _yi = (int) ( ( _y - min_y_ ) / res_ );
  _zi = (int) ( ( _z - min_z_ ) / res_ );
}

/**
 * @function WorldToIndex
 */
int ReachGraph::worldToIndex( const double &_x, 
			 const double &_y, 
			 const double &_z ) {
  
  int xi, yi, zi;
  worldToVertex( _x, _y, _z, xi, yi, zi );
  return ref( xi, yi, zi );
}

/**
 * @function VertexToWorld
 */
void ReachGraph::vertexToWorld( const int &_xi, const int &_yi, const int &_zi,  
			   double &_x, double &_y, double &_z ) {

  // 0 <= _vx < numVx && 0 <= _vy < numVy && 0 <= _vz < numVz
  _x = min_x_ + res_*_xi;
  _y = min_y_ + res_*_yi;
  _z = min_z_ + res_*_zi;
}

/**
 * @function IndexToVertex
 */
void ReachGraph::indexToVertex( const int &_ind,
			   int &_x, int &_y, int &_z ) {
 
  int remaining;
  _x = _ind / step_yz_;
  remaining = _ind % step_yz_;
  _y = remaining / step_z_;
  _z = remaining % step_z_;
}

/**
 * @function createBox
  */
 /*
void ReachGraph::createBox( double _ox, double _oy, double _oz,
		       double _lx, double _ly, double _lz,
		       int _type ) {
  
  int b1x; int b1y; int b1z;
  int b2x; int b2y; int b2z;

  WorldToVertex( _ox, _oy, _oz, b1x, b1y, b1z );
  WorldToVertex( (_lx + _ox), (_ly +_oy), (_lz + _oz),
		 b2x, b2y, b2z );

  for( int i = b1x; i < b2x; i++ ) {
    for( int j = b1y; j < b2y; j++ ) {
      for( int k = b1z; k < b2z; k++ ) {
	mV[ref(i,j,k)] = _type;
      }
    }
  }
}*/

/**
 * @function setState  
 */
void ReachGraph::setState( int _xi, int _yi, int _zi, ReachData _rd )
{
    points_[ref(_xi, _yi, _zi)] = _rd;
}


/**
 * @function getPCD
 */
sensor_msgs::msg::PointCloud2 ReachGraph::getPCD( ReachDataState _state, int _r, int _g, int _b )
{
  ReachData *v;
  v = &points_[0];
  
  int max_sols = 0;
  int min_sols = 100000;

  // Get how many obstacle vertices there are in the graph
  int count = 0;
  for( int i = 0; i < num_points_; ++i ) {
    if( v->state == _state ) {
      count++;
      if(v->num_sols > max_sols)
        max_sols = v->num_sols;
      if(v->num_sols < min_sols)
        min_sols = v->num_sols;
    }
    v++;
  }

  RCLCPP_WARN(rclcpp::get_logger("getPCD"), "NUmber of solutions range from %d to %d ", min_sols, max_sols);

  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.frame_id = chain_info_.root_link;
  cloud.width = count;
  cloud.height = 1;
  cloud.is_dense = false;
 
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(count);

  // iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud, "b");

  // Enter vertices in the graph
  int xi, yi, zi;
  double x, y, z;
  v = &points_[0];

  for( int i = 0; i < num_points_; ++i ) {
    if( v->state == _state ) {
 
      indexToVertex( i, xi, yi, zi );
      vertexToWorld( xi, yi, zi, x, y, z );

      double ratio = (double) v->num_sols / (double) max_sols;
      *out_x = x;
      *out_y = y;
      *out_z = z;
      *out_r = (int)( 255.0*( modulo( 2*(1-ratio), 1.0) )); 
      *out_g = (int)( 255*( modulo( 2*ratio, 1.0) )); 
      *out_b = 0; 

      ++out_x; ++out_y; ++out_z;
      ++out_r; ++out_g; ++out_b;
    }
    v++;
  }
 
  return cloud;
}

/**
 * @function getPCD
 */
sensor_msgs::msg::PointCloud2 ReachGraph::debugSamples(int _xi, int _yi, int _zi)
{
  int N = 64;
  sensor_msgs::msg::PointCloud2 cloud;

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
  createSphereSamplesVoxel(_xi, _yi, _zi, frames, N);
  for( int i = 0; i < N; ++i ) { 
      float xd, yd, zd;
      xd = (float)frames[i].translation()(0);
      yd = (float)frames[i].translation()(1);
      zd = (float)frames[i].translation()(2);
      printf("Got xd yd zd: %f %f %f \n", xd, yd, zd);
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

/**
 * @brief createSphereSamplesVoxel
 */
void ReachGraph::createSphereSamplesVoxel(const int &_xi, 
                                const int &_yi, 
                                const int &_zi,
                                std::vector<Eigen::Isometry3d> &_frames, 
                                const int &_n)
{
  _frames.clear();

  double x, y, z;
  double dx, dy, dz;

  vertexToWorld(_xi, _yi, _zi, x, y, z);

  double theta_k, phi_k, h_k;
  double r= res_/2.0;

  for(int k = 1; k <= _n; ++k)
  {

   h_k = -1.0 + 2.0*(double)(k-1)/(double)(_n-1);
   theta_k = acos(h_k);

   if(k == 1 || k == _n)
    phi_k = 0;
   else
    phi_k = modulo(phi_k + 3.6/sqrt(_n) *1/sqrt(1.0 - h_k*h_k), (2*M_PI));
  
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