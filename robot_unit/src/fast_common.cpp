#include <robot_unit/fast_common.h>
#include <tf2_eigen/tf2_eigen.hpp>

CollisionGeometryData::CollisionGeometryData() :
  shape_index_(0)
{

}

FCLGeometry::FCLGeometry()
{}

FCLGeometry::FCLGeometry(fcl::CollisionGeometryd* _collision_geometry,
                         std::string _id) :
  collision_geometry_(_collision_geometry)
{
  CollisionGeometryDataPtr cgd(new CollisionGeometryData);
  cgd->id_ = _id;

  collision_geometry_data_ = cgd;
  collision_geometry_->setUserData(collision_geometry_data_.get());
}

FCLGeometry::FCLGeometry(fcl::CollisionGeometryd* _collision_geometry,
                         const std::string &_id,
                         const int &_shape_index,
                         const fcl::Transform3d &_local_tf) :
  collision_geometry_(_collision_geometry)
{
  CollisionGeometryDataPtr cgd(new CollisionGeometryData);
  cgd->id_ = _id;
  cgd->shape_index_ = _shape_index;
  cgd->local_tf_ = _local_tf;

  collision_geometry_data_ = cgd;
  collision_geometry_->setUserData(collision_geometry_data_.get());
}

void FCLObject::registerTo(fcl::BroadPhaseCollisionManagerd* _manager)
{
  std::vector<fcl::CollisionObjectd*> cos(collision_objects_.size());
  for (std::size_t i = 0; i < collision_objects_.size(); ++i)
    cos[i] = collision_objects_[i].get();
  if (!cos.empty())
    _manager->registerObjects(cos);
}


void FCLObject::unregisterFrom(fcl::BroadPhaseCollisionManagerd* _manager)
{
  for (auto &co : collision_objects_)
    _manager->unregisterObject(co.get());
}

int FCLObject::unregisterShape(const std::string &_id, 
                               std::shared_ptr<fcl::BroadPhaseCollisionManagerd> _manager,
                               const bool &_update_manager)
{
  std::vector<int> indices;
  getShapeIndices(_id, indices);

  for(auto idx : indices)
    _manager->unregisterObject( collision_objects_[idx].get() );
  
  if(_update_manager)
    _manager->update();

  return indices.size();
}

void FCLObject::clear()
{
  for(auto &a : collision_objects_)
    a.reset();

  for(auto &a : collision_geometry_)
    a.reset();

  collision_objects_.clear();

  collision_geometry_.clear();
}

/**
 * @function getObjectIndices
 * @brief Given the name of an element, return the indices of the shapes stored
 * indices is usually of size 1. It only gets to be > 1 when you are dealing with a urdf's link
 * with multiple collision objects, which is not very common
 */
bool FCLObject::getShapeIndices(const std::string &_id, std::vector<int> &_indices)
{
  _indices.clear();

  for(int i = 0; i < collision_geometry_.size(); ++i)
  {
    if( collision_geometry_[i]->collision_geometry_data_->id_ == _id )
      _indices.push_back(i);    
  }

  return (_indices.size() > 0);
}

void FCLObject::addShape(fcl::CollisionObjectd* _fcl_obj, FCLGeometryConstPtr _g)
{
  this->collision_objects_.push_back(std::shared_ptr<fcl::CollisionObjectd>(_fcl_obj));
  this->collision_geometry_.push_back(_g);
}

int FCLObject::deleteShape(const std::string &_id)
{
  std::vector<int> indices;
  getShapeIndices(_id, indices);

  // Indices come in increasing order. Delete them from back to front as to not alter indices 
  // as we delete
  if(indices.size() == 0)
    return 0;

  for(int i = indices.size() -1; i == 0; i--)
  {
    collision_objects_.erase(collision_objects_.begin() + i);
    collision_geometry_.erase(collision_geometry_.begin() + i);
  }

  return indices.size();
}

bool FCLObject::hasShape(const std::string &_id)
{
  for (auto oi :  this->collision_geometry_)
    if (oi->collision_geometry_data_->id_ == _id)
      return true;

  return false;
}

//////////////////////////////////////////////////////////

bool SimpleCollisionFunction(fcl::CollisionObjectd* o1,
                             fcl::CollisionObjectd* o2,
                             void* data)
{

  assert(data != nullptr);

  auto* cdata = static_cast<SimpleCollisionData*>(data);

  const fcl::CollisionRequestd& request = cdata->request;
  fcl::CollisionResultd& result = cdata->result;

  const CollisionGeometryData* cd1 = static_cast<const CollisionGeometryData*>(o1->collisionGeometry()->getUserData());
  const CollisionGeometryData* cd2 = static_cast<const CollisionGeometryData*>(o2->collisionGeometry()->getUserData());

  if (cdata->done)
    return true;

  // If collision objects belong to the same link, ignore check
  if (cd1->id_ == cd2->id_)
    return false;

  if (cdata->acm_)
  {
    if (cdata->acm_->getEntry(cd1->id_, cd2->id_))
      return false;
  }


  fcl::collide(o1, o2, request, result);

  if (!request.enable_cost && result.isCollision() &&
      result.numContacts() >= request.num_max_contacts)
  {
    cdata->done = true;
    //printf("\t * Found collision with objects %s and %s. Result: %d \n",
    //       cd1->id_.c_str(), cd2->id_.c_str(), result.isCollision());
  }

  return cdata->done;
}

//////////////////////////////////////////////////////////

bool MultiCollisionFunction(fcl::CollisionObjectd* o1,
                            fcl::CollisionObjectd* o2,
                            void* data)
{

  assert(data != nullptr);

  auto* cdata = static_cast<MultiCollisionData*>(data);

  const fcl::CollisionRequestd& request = cdata->request;
  fcl::CollisionResultd& result = cdata->result;

  if (cdata->done)
    return true;

  fcl::collide(o1, o2, request, result);

  if (!request.enable_cost && result.isCollision() &&
      result.numContacts() >= request.num_max_contacts)
  {
    // Return poses and done=true
    cdata->pose_1 = tf2::toMsg(o1->getTransform());
    cdata->pose_2 = tf2::toMsg(o2->getTransform());
      
    cdata->done = true;
  }
  return cdata->done;
}

