/**
 * @file Graph.h 
 * @author A. Huaman Quispe
 * @date 2012-08-16
 */

#ifndef __REACH_DATA_H__
#define __REACH_DATA_H__

#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <robot_unit/robot_entity.h>

namespace reachability_description
{

enum ReachDataState {
  OBSTACLE = 0,
  FILLED,
  NO_FILLED,
  COLLISION
};

struct  ReachData
{
  ReachDataState state;  

  double metric;
  int num_sols;
};

/**
 * @class ReachGraph
 */
class ReachGraph {

 public:
  ReachGraph( const ChainInfo &_chain_info, double _min_x, double _min_y, double _min_z,
	          double _max_x, double _max_y, double _max_z,
	          double _resolution, 
	          ReachData _default);
  ~ReachGraph();

  void worldToVertex( const double &_x, const double &_y, const double &_z,
		              int &_xi, int &_yi, int &_zi );
  
  int worldToIndex( const double &_x, const double &_y, const double &_z );

  void vertexToWorld( const int &_xi, const int &_yi, const int &_zi, 
		      double &_x, double &_y, double &_z ); 

  void indexToVertex( const int &_ind, int &_x, int &_y, int &_z );

/*  void createBox( double _ox, double _oy, double _oz,
		  double _lx, double _ly, double _lz,
		  int _type = OBSTACLE );
*/
  sensor_msgs::msg::PointCloud2 getPCD( ReachDataState _state, int _r, int _g, int _b );

  inline int ref( int _xi, int _yi, int _zi );
  inline int getNumPoints();
  inline int getNumX();
  inline int getNumY();
  inline int getNumZ();
  inline double getResolution() const;
  inline bool isValid( int _xi, int _yi, int _zi );
  inline ReachData getState( int _ind );
  inline ReachData getState( int _xi, int _yi, int _zi );
  void setState( int _xi, int _yi, int _zi, ReachData _rd );

  inline double getMinX() const { return min_x_; }
  inline double getMinY() const { return min_y_; }
  inline double getMinZ() const { return min_z_; }

  inline double getMaxX() const { return max_x_; }
  inline double getMaxY() const { return max_y_; }
  inline double getMaxZ() const { return max_z_; }


 private:

  int num_points_; //< Number of total vertices
  ReachData* points_;  //< Vertices
  double min_x_; double min_y_; double min_z_; //< Origin coordinates for each direction
  double max_x_; double max_y_; double max_z_; //< Max value for each direction
  double res_;  //< Resolution
  int num_x_; int num_y_; int num_z_; //< Num vertices at each direction
  int step_yz_;
  int step_z_;

  // Chain info
  ChainInfo chain_info_;
};

/**
 * @function getNumV
 */
inline double ReachGraph::getResolution() const {
  return res_;
}


/**
 * @function getNumV
 */
inline int ReachGraph::getNumPoints() {
  return num_points_;
}

/**
 * @function getNumVx
 */
inline int ReachGraph::getNumX() {
  return num_x_;
}

/**
 * @function getNumVy
 */
inline int ReachGraph::getNumY() {
  return num_y_;
}

/**
 * @function getNumVz
 */
inline int ReachGraph::getNumZ() {
  return num_z_;
}
/**
 * @function ref
 */
inline int ReachGraph::ref( int _xi, int _yi, int _zi ) {
  return _xi*step_yz_ + _yi*step_z_ + _zi;
}


/**
 * @function isValid
 */
inline bool ReachGraph::isValid( int _xi, int _yi, int _zi ) {
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
inline ReachData ReachGraph::getState( int _ind ) {
  return points_[ _ind ];
}

/** 
 * @function getState
 */
inline ReachData ReachGraph::getState( int _xi, int _yi, int _zi ) {
  return points_[ ref(_xi, _yi, _zi) ];
}

} // namespace reachability_description

#endif /** __REACH_DATA_H__ */