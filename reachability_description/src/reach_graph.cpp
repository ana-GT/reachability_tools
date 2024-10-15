/**
 * @file Graph.cpp
 * @author A. Huaman Q.
 * @date 2012 / 08/ 16
 */
#include <reachability_description/reach_graph.h>
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
 * @function ReachGraph
 */
ReachGraph::ReachGraph() {
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
 * @function initialize
 */
bool ReachGraph::initialize( const reachability_msgs::msg::ChainInfo &_chain_info, 
                             double _min_x, double _min_y, double _min_z,
	                     double _max_x, double _max_y, double _max_z,
	                     const double &_resolution, 
                             const uint16_t &_voxel_samples,
	                     const reachability_msgs::msg::ReachData &_default )
{
  chain_info_ = _chain_info;
 
  params_.min_x = _min_x;
  params_.min_y = _min_y;
  params_.min_z = _min_z;
  params_.max_x = _max_x;
  params_.max_y = _max_y;
  params_.max_z = _max_z;
  params_.resolution = _resolution;
  params_.num_voxel_samples = _voxel_samples;

  calculateDims();

  points_ = new reachability_msgs::msg::ReachData[num_points_];
  std::fill( points_, points_ + num_points_, _default );
  return true;
}

/**
 * @function initialize
 * @brief Fill class with data from message
 */
bool ReachGraph::initialize(const reachability_msgs::msg::ReachGraph &_msg)
{
  chain_info_ = _msg.chain_info;
  params_ = _msg.params;

  calculateDims();
  points_ = new reachability_msgs::msg::ReachData[num_points_];
  reachability_msgs::msg::ReachData default_data;
  default_data.state = reachability_msgs::msg::ReachData::NO_FILLED;

  std::fill( points_, points_ + num_points_, default_data );

  if(_msg.points.size() != num_points_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ReachGraph"), "Loaded reachability does not match: %ld vs %d ", _msg.points.size(), num_points_); 
    return false;
  } 
  
  RCLCPP_INFO(rclcpp::get_logger("ReachGraph"), "Loading %ld points ", _msg.points.size());   

  for(int i = 0; i < num_points_; ++i)
    points_[i] = _msg.points[i];
 
  return true;
}


/**
 * @function calculateDims 
 * @brief Calculate number of discrete units for X, Y and Z axes
 */
void ReachGraph::calculateDims()
{
  num_x_ = round( ( params_.max_x - params_.min_x ) / params_.resolution );
  num_y_ = round( ( params_.max_y - params_.min_y ) / params_.resolution );
  num_z_ = round( ( params_.max_z - params_.min_z ) / params_.resolution );

  step_yz_ = num_y_*num_z_;
  step_z_ = num_z_;

  num_points_ = num_x_*num_y_*num_z_;
}

/**
 * @function storeGraph 
 */
bool ReachGraph::toMsg(reachability_msgs::msg::ReachGraph &_graph)
{
  // Reset just in case
  _graph.points.clear();

  // Fill params
  _graph.chain_info = this->chain_info_;
  _graph.params = this->params_;

  // Fill points
  reachability_msgs::msg::ReachData *v;
  v = &points_[0];
  
  for( int i = 0; i < num_points_; ++i ) {
    reachability_msgs::msg::ReachData data = *v;
    _graph.points.push_back(data);
    v++;
  }

  return true;
}


/**
 * @function WorldToVertex
 */
bool ReachGraph::worldToVertex( const double &_x, const double &_y, const double &_z,
    			         int &_xi, int &_yi, int &_zi ) const {

  // mOX <= _wx <= mMaxX && mOX <= _wx <= mMaxY && mOY <= _wz <= mMaxZ
  if(_x < params_.min_x || _x > params_.max_x ||
  _y < params_.min_y || _y > params_.max_y ||
  _z < params_.min_z || _z > params_.max_z )
    return false;

  // floor effect expected: Round to minimum integer
  _xi = floor( ( _x - params_.min_x ) / params_.resolution );
  _yi = floor( ( _y - params_.min_y ) / params_.resolution );
  _zi = floor( ( _z - params_.min_z ) / params_.resolution );

  return true;
}

/**
 * @function WorldToIndex
 */
int ReachGraph::worldToIndex( const double &_x, 
			       const double &_y, 
			       const double &_z ) const {
  
  int xi, yi, zi;
  worldToVertex( _x, _y, _z, xi, yi, zi );
  return ref( xi, yi, zi );
}

/**
 * @function VertexToWorld
 * @brief Vertex is the center of the voxel
 */
bool ReachGraph::vertexToWorld( const int &_xi, const int &_yi, const int &_zi,  
			         double &_x, double &_y, double &_z ) const {

  // 0 <= _vx < numVx && 0 <= _vy < numVy && 0 <= _vz < numVz
  if(!isValid(_xi, _yi, _zi))
     return false;

  double r = 0.5 * params_.resolution;

  _x = params_.min_x + params_.resolution*_xi + r;
  _y = params_.min_y + params_.resolution*_yi + r;
  _z = params_.min_z + params_.resolution*_zi + r;
  return true;
}

/**
 * @function IndexToVertex
 */
void ReachGraph::indexToVertex( const int &_ind,
			         int &_x, int &_y, int &_z ) const {
 
  int remaining;
  _x = _ind / step_yz_;
  remaining = _ind % step_yz_;
  _y = remaining / step_z_;
  _z = remaining % step_z_;
}

/**
 * @function setState  
 */
void ReachGraph::setState( int _xi, int _yi, int _zi, const reachability_msgs::msg::ReachData &_rd )
{
    points_[ref(_xi, _yi, _zi)] = _rd;
}

/**
 * @function setPlanEquationCoefficients
 */
bool ReachGraph::setPlaneEquationCoefficients(const std::string &_plane, 
                                              const double &_plane_dist,
                                              double &_nx, double &_ny, double &_nz, double &_d)
{
  if(_plane == std::string("XY+"))
    { _nx = 0; _ny = 0; _nz = -1.0; _d = _plane_dist; }
  else if(_plane == std::string("XY-"))
    { _nx = 0; _ny = 0; _nz = 1.0; _d = -_plane_dist; } 
  else if(_plane == std::string("XZ+"))
    { _nx = 0; _ny = -1.0; _nz = 0.0; _d = _plane_dist; }
  else if(_plane == std::string("XZ-"))
    { _nx = 0; _ny = 1.0; _nz = 0.0; _d = -_plane_dist; } 
  else if(_plane == std::string("YZ+"))
    { _nx = -1.0; _ny = 0.0; _nz = 0.0; _d = _plane_dist; }
  else if(_plane == std::string("YZ-"))
    { _nx = 1.0; _ny = 0; _nz = 0.0; _d = -_plane_dist; } 
  else if(_plane == std::string("FULL"))
  { _nx = 0; _ny = 0; _nz = 0; _d = 0; }
  else
  {
     RCLCPP_ERROR(rclcpp::get_logger("ReachGraph"), " plane parameter is not set up with a valid string!");
     return false;
  }
  
  return true;
}

/**
 * @function getPCD
 */
sensor_msgs::msg::PointCloud2 ReachGraph::getPCD( const std::string &_plane,
                                                  const double &_plane_dist )
{
  RCLCPP_WARN(rclcpp::get_logger("ReachGraph"), "Plane type: %s dist: %f ", _plane.c_str(), _plane_dist);
  reachability_msgs::msg::ReachData *v;
  v = &points_[0];
  
  int max_sols = 0;
  int min_sols = 100000;

  int xi, yi, zi;
  double x, y, z;

  double nx, ny, nz, d;
  setPlaneEquationCoefficients(_plane, _plane_dist, nx, ny, nz, d);

  // Get how many points are above the given plane
  int count = 0;
  for( int i = 0; i < num_points_; ++i ) {
    if( v->state == reachability_msgs::msg::ReachData::FILLED ) {

      indexToVertex( i, xi, yi, zi );
      vertexToWorld( xi, yi, zi, x, y, z );

      if( nx*x + ny*y + nz*z +d >= 0.0)
        count++;

        if(v->samples.size() > max_sols)
          max_sols = v->samples.size();
        if(v->samples.size() < min_sols)
          min_sols = v->samples.size();
    }
    v++;
  } // for i

  RCLCPP_WARN(rclcpp::get_logger("ReachGraph"), "Number of solutions range from %d to %d ", min_sols, max_sols);

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
  v = &points_[0];

  for( int i = 0; i < num_points_; ++i ) {
    if( v->state == reachability_msgs::msg::ReachData::FILLED ) {

      indexToVertex( i, xi, yi, zi );
      vertexToWorld( xi, yi, zi, x, y, z );

      if( nx*x + ny*y + nz*z +d >= 0.0)
      {
        double ratio = (double) v->samples.size() / (double) params_.num_voxel_samples;
        *out_x = x;
        *out_y = y;
        *out_z = z;
        //*out_r = (int)( 255.0*( modulo( 2*(1-ratio), 1.0) )); 
        //*out_g = (int)( 255*( modulo( 2*ratio, 1.0) )); 
        double red = ratio > 0.5? 1.0 - 2.0*(ratio - 0.5) : 1.0;
        double green = ratio > 0.5? 1.0 : 2.0*ratio;
        *out_r = (int)(red*255);
        *out_g = (int)(green*255);
        *out_b = 0; 

        ++out_x; ++out_y; ++out_z;
        ++out_r; ++out_g; ++out_b;
      } // if nx, ny, nz
    }
    v++;
  }
 
  return cloud;
}


/**
 * @function getPCD
 */
sensor_msgs::msg::PointCloud2 ReachGraph::getPCDHigherThan(const double &_ratio)
{
  RCLCPP_WARN(rclcpp::get_logger("ReachGraph"), "PCD higher than");
  reachability_msgs::msg::ReachData *v;
  v = &points_[0];
  
  int max_sols = 0;
  int min_sols = 100000;

  int xi, yi, zi;
  double x, y, z;

  // Get how many reachability samples have a metric value higher than a ratio of the maximum value
  int count = 0;
  int ratio_above = floor(_ratio *  params_.num_voxel_samples);
  for( int i = 0; i < num_points_; ++i ) {
    if( v->state == reachability_msgs::msg::ReachData::FILLED ) {

      if(v->samples.size() >= ratio_above)
        count++;
    }
    v++;
  }

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
  v = &points_[0];

  for( int i = 0; i < num_points_; ++i ) {
    if( v->state == reachability_msgs::msg::ReachData::FILLED ) {

      indexToVertex( i, xi, yi, zi );
      vertexToWorld( xi, yi, zi, x, y, z );

      if(v->samples.size() >= ratio_above)
      {
        *out_x = x;
        *out_y = y;
        *out_z = z;
        *out_r = 20;
        *out_g = 255;
        *out_b = 20; 

        ++out_x; ++out_y; ++out_z;
        ++out_r; ++out_g; ++out_b;
      } // 
    } // if
    v++;
  }
 
  return cloud;
}


