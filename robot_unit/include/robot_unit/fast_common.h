#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision.h>

#include <robot_unit/fast_allowed_collision_matrix.h>

#include <geometry_msgs/msg/pose.hpp>

struct CollisionGeometryData
{
  CollisionGeometryData();

  const std::string& getId() const;

  //
  std::string id_;
  int shape_index_;
  fcl::Transform3d local_tf_;
};

typedef std::shared_ptr<CollisionGeometryData> CollisionGeometryDataPtr;

/**
 * @struct FCLGeometry
 */
struct FCLGeometry
{
  FCLGeometry();
  FCLGeometry(fcl::CollisionGeometryd* collision_geometry,
              std::string _id);
  FCLGeometry(fcl::CollisionGeometryd* collision_geometry,
              const std::string &_id,
              const int &_shape_index,
              const fcl::Transform3d &_local_tf);

  std::shared_ptr<fcl::CollisionGeometryd> collision_geometry_; // fcl::Geometry
  CollisionGeometryDataPtr collision_geometry_data_; // "Identifier", one per each collision_geometry_
};

typedef std::shared_ptr<FCLGeometry> FCLGeometryPtr;
typedef std::shared_ptr<const FCLGeometry> FCLGeometryConstPtr;

/**
 * @struct FCLObject
 */
struct FCLObject
{
  void registerTo(fcl::BroadPhaseCollisionManagerd* _manager);
  void unregisterFrom(fcl::BroadPhaseCollisionManagerd* _manager);
  void clear();

  bool getShapeIndices(const std::string &_id, std::vector<int> &_indices);
  
  bool hasShape(const std::string &_id);
  void addShape(fcl::CollisionObjectd* _fcl_obj, FCLGeometryConstPtr _g);
  int deleteShape(const std::string &_id);

  int unregisterShape(const std::string &_id, 
                      std::shared_ptr<fcl::BroadPhaseCollisionManagerd> _manager,
                      const bool &_update_manager =  true);

  std::vector< std::shared_ptr<fcl::CollisionObjectd> > collision_objects_;
  std::vector<FCLGeometryConstPtr> collision_geometry_;
};

/**
 * @struct SimpleCollisionData
 */
struct SimpleCollisionData
{
  fcl::CollisionRequestd request;
  fcl::CollisionResultd result;

  bool done{false};
  const robot_unit::AllowedCollisionMatrix* acm_;
};

/**
 * @function SimpleCollisionFunction
 */
bool SimpleCollisionFunction(fcl::CollisionObjectd* o1,
                             fcl::CollisionObjectd* o2,
                             void* data);


/**
 * @struct MultiCollisionData
 */
struct MultiCollisionData
{
  fcl::CollisionRequestd request;
  fcl::CollisionResultd result;

  geometry_msgs::msg::Pose pose_1; // pose of specific colliding body in robot 1
  geometry_msgs::msg::Pose pose_2; // pose of specific colliding body in robot 2

  bool done{false};

};

/**
 * @function MultiCollisionFunction
 */
bool MultiCollisionFunction(fcl::CollisionObjectd* o1,
                            fcl::CollisionObjectd* o2,
                            void* data);
