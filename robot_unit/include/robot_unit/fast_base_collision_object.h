#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>

#include <robot_unit/fast_common.h>

namespace robot_unit
{

/**
 * @class BaseCollisionObject
 */
class BaseCollisionObject
{
public:
  BaseCollisionObject();
  ~BaseCollisionObject();

  virtual bool init(const std::string &_ref_frame);
  bool init_(const std::string &_ref_frame);

  bool setup();

/*  bool collide(std::shared_ptr<BaseCollisionObject> _obj2,
               plummrs_msgs::CollisionInfo &_collision_info);*/
  bool distance(std::shared_ptr<BaseCollisionObject> _obj2,
                double &_min_distance);               
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> getManagerPtr()
  {
    return manager_;
  }

  void addShape(fcl::CollisionObjectd* _fcl_obj, FCLGeometryConstPtr _g);
  bool hasShape(const std::string &_id);
  void deleteShape(const std::string &_id);

protected:

  // Manager
  // std::unique_ptr
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager_;
  std::string reference_frame_;

  FCLObject object_;

};


} // namespace robot_unit
