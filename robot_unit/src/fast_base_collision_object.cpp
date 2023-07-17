/**
 * @file fast_base_collision_object.cpp
 */
#include <robot_unit/fast_base_collision_object.h>

using namespace robot_unit;

BaseCollisionObject::BaseCollisionObject()
{
  manager_.reset(new fcl::DynamicAABBTreeCollisionManagerd());

}

BaseCollisionObject::~BaseCollisionObject()
{
  manager_->clear();
  manager_.reset();
  object_.clear();
}

bool BaseCollisionObject::init(const std::string &_ref_frame)
{
  return init_(_ref_frame);
}

bool BaseCollisionObject::init_(const std::string &_ref_frame)
{
  reference_frame_ = _ref_frame;
  return true;
}

bool BaseCollisionObject::setup()
{
  manager_->setup();
  return true;
}

void BaseCollisionObject::addShape(fcl::CollisionObjectd* _fcl_obj, FCLGeometryConstPtr _g)
{
  // Add collision and its info in the FCLObject
  object_.addShape(_fcl_obj, _g);

  // Register to manager
  manager_->registerObject(_fcl_obj);

}

bool BaseCollisionObject::hasShape(const std::string &_id)
{

  // Delete FCL objects
  return object_.hasShape(_id);
}


void BaseCollisionObject::deleteShape(const std::string &_id)
{
  // Unregister to manager
  bool update_manager = true;
  object_.unregisterShape(_id, manager_, update_manager);

  // Delete FCL objects
  object_.deleteShape(_id);
}

/*
bool BaseCollisionObject::collide(std::shared_ptr<BaseCollisionObject> _obj2,
                                  plummrs_msgs::CollisionInfo &_collision_info)
{
  MultiCollisionData cd;
  manager_->collide(_obj2->getManagerPtr().get(), &cd,
                    MultiCollisionFunction);

  if (cd.result.isCollision())
  {
    if (cd.result.numContacts() > 0)
    {
      fcl::Contact<double> ci = cd.result.getContact(0);

      _collision_info.pose_1.header.frame_id = reference_frame_;
      _collision_info.pose_1.pose = cd.pose_1;
      _collision_info.pose_2.header.frame_id = reference_frame_;
      _collision_info.pose_2.pose = cd.pose_2;

      const CollisionGeometryData* cd1 = static_cast<const CollisionGeometryData*>(ci.o1->getUserData());
      const CollisionGeometryData* cd2 = static_cast<const CollisionGeometryData*>(ci.o2->getUserData());

      _collision_info.object_1 = cd1->id_.c_str();
      _collision_info.object_2 = cd2->id_.c_str();
    }
  }
  return cd.result.isCollision();
}*/

bool BaseCollisionObject::distance(std::shared_ptr<BaseCollisionObject> _obj2,
                                   double &_min_distance)
{
  fcl::DefaultDistanceData<double> dd;
  manager_->distance(_obj2->getManagerPtr().get(), &dd,
                    fcl::DefaultDistanceFunction);
  _min_distance = dd.result.min_distance;

  return true;
}