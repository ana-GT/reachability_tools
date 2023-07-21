/**
 * @file reach_data.h
 * @author A. Huaman Quispe
 * @date 2012-08-16
 */

#ifndef __REACH_DATA_H__
#define __REACH_DATA_H__

#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <robot_unit/robot_entity.h>
#include <Eigen/Geometry>

#include <reachability_msgs/msg/reach_params.hpp>
#include <reachability_msgs/msg/reach_graph.hpp>
#include <reachability_msgs/msg/reach_data.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace reachability_description
{

/**
 * @class ReachGraph
 */
class ReachGraph {

 public:
  ReachGraph( const reachability_msgs::msg::ChainInfo &_chain_info, 
            double _min_x, double _min_y, double _min_z,
	          double _max_x, double _max_y, double _max_z,
	          const double &_resolution, 
            const uint16_t &_voxel_samples,
	          const reachability_msgs::msg::ReachData &_default);
  
  ReachGraph(const reachability_msgs::msg::ReachGraph &_msg);

  ~ReachGraph();

  bool worldToVertex( const double &_x, const double &_y, const double &_z,
		              int &_xi, int &_yi, int &_zi ) const;
  
  int worldToIndex( const double &_x, const double &_y, const double &_z ) const;

  bool vertexToWorld( const int &_xi, const int &_yi, const int &_zi, 
		      double &_x, double &_y, double &_z ) const; 

  void indexToVertex( const int &_ind, int &_x, int &_y, int &_z ) const;

/*  void createBox( double _ox, double _oy, double _oz,
		  double _lx, double _ly, double _lz,
		  int _type = OBSTACLE );
*/
  void createSphereSamplesVoxel(const int &_xi, 
                                const int &_yi, 
                                const int &zi,
                                std::vector<Eigen::Isometry3d> &_frames) const;

  sensor_msgs::msg::PointCloud2 getPCD( const uint8_t &_state, 
                                        int _r, int _g, int _b );

  sensor_msgs::msg::PointCloud2 debugSamples(int _xi, int _yi, int _zi);

  bool toMsg(reachability_msgs::msg::ReachGraph &_graph);

  inline int ref( int _xi, int _yi, int _zi ) const;
  inline int getNumPoints() const;
  inline int getNumX() const;
  inline int getNumY() const;
  inline int getNumZ() const;
  inline double getResolution() const;
  inline bool isValid( int _xi, int _yi, int _zi ) const;
  inline reachability_msgs::msg::ReachData getState( int _ind );
  inline reachability_msgs::msg::ReachData getState( int _xi, int _yi, int _zi );
  void setState( int _xi, int _yi, int _zi, const reachability_msgs::msg::ReachData &_rd );

  inline int getNumVoxelSamples() const { return params_.num_voxel_samples; }

  inline double getMinX() const { return params_.min_x; }
  inline double getMinY() const { return params_.min_y; }
  inline double getMinZ() const { return params_.min_z; }

  inline double getMaxX() const { return params_.max_x; }
  inline double getMaxY() const { return params_.max_y; }
  inline double getMaxZ() const { return params_.max_z; }

 private:

  void calculateDims();


  reachability_msgs::msg::ChainInfo chain_info_;
  reachability_msgs::msg::ReachParams params_;

  reachability_msgs::msg::ReachData* points_;  //< Vertices

  int num_points_; //< Number of total vertices
  int num_x_; int num_y_; int num_z_; //< Num vertices at each direction
  int step_yz_;
  int step_z_;

};

/**
 * @function getNumV
 */
inline double ReachGraph::getResolution() const {
  return params_.resolution;
}


/**
 * @function getNumV
 */
inline int ReachGraph::getNumPoints() const{
  return num_points_;
}

/**
 * @function getNumVx
 */
inline int ReachGraph::getNumX() const{
  return num_x_;
}

/**
 * @function getNumVy
 */
inline int ReachGraph::getNumY() const{
  return num_y_;
}

/**
 * @function getNumVz
 */
inline int ReachGraph::getNumZ() const{
  return num_z_;
}
/**
 * @function ref
 */
inline int ReachGraph::ref( int _xi, int _yi, int _zi ) const {
  return _xi*step_yz_ + _yi*step_z_ + _zi;
}


/**
 * @function isValid
 */
inline bool ReachGraph::isValid( int _xi, int _yi, int _zi ) const {
  if( _xi >= 0 && _xi < num_x_ &&
      _yi >= 0 && _yi < num_y_ && 
      _zi >= 0 && _zi < num_z_ ) {
    return true;
  }
  return false;
}

/**
 * @function getState
 */
inline reachability_msgs::msg::ReachData ReachGraph::getState( int _ind ) {
  return points_[ _ind ];
}

/** 
 * @function getState
 */
inline reachability_msgs::msg::ReachData ReachGraph::getState( int _xi, int _yi, int _zi ) {
  return points_[ ref(_xi, _yi, _zi) ];
}

} // namespace reachability_description

#endif /** __REACH_DATA_H__ */