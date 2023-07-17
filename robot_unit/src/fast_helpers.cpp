/**
 * @file fast_helpers.cpp
 * @brief
 */
#include <robot_unit/fast_helpers.h>
#include <octomap_msgs/conversions.h>
#include <fcl/geometry/octree/octree.h>

/**
 * @function urdfCollisionToFclTf
 */
fcl::Transform3d urdfCollisionToFclTf(const urdf::CollisionSharedPtr &_coll)
{
  fcl::Transform3d tfx;
  tfx.setIdentity();

  fcl::Quaterniond q(_coll->origin.rotation.w,
                     _coll->origin.rotation.x,
                     _coll->origin.rotation.y,
                     _coll->origin.rotation.z);

  if (q.norm() == 0)
    q = fcl::Quaterniond(1, 0, 0, 0);
  q.normalize();

  tfx.translation() = fcl::Vector3d(_coll->origin.position.x,
                                    _coll->origin.position.y,
                                    _coll->origin.position.z);
  tfx.linear() = q.matrix();
  return tfx;
}

/**
 * @function urdfCollisionToFCL
 * @brief Get a fcl::CollisionGeometryd* pointer from a urdf::CollsionShared pointer
 */
fcl::CollisionGeometryd* urdfCollisionToFcl(const urdf::CollisionSharedPtr &_coll,
    double _padding, double _scale)
{
  fcl::CollisionGeometryd* cg = nullptr;

  shapes::Shape* shape = nullptr;
  shape = constructShape(_coll->geometry.get());

  if (!shape)
    return cg;

  shape->scale(_scale);
  shape->padd(_padding);


  switch (shape->type)
  {
  case shapes::CYLINDER:
  {
    cg = new fcl::Cylinderd(dynamic_cast<shapes::Cylinder*>(shape)->radius,
                            dynamic_cast<shapes::Cylinder*>(shape)->length);
    break;
  }
  case shapes::SPHERE:
  {
    cg = new fcl::Sphered(dynamic_cast<shapes::Sphere*>(shape)->radius);
    break;
  }
  case shapes::BOX:
  {
    cg = new fcl::Boxd(dynamic_cast<shapes::Box*>(shape)->size[0],
                       dynamic_cast<shapes::Box*>(shape)->size[1],
                       dynamic_cast<shapes::Box*>(shape)->size[2]);
    break;
  }
  case shapes::MESH:
  {
    cg = meshToBvh(dynamic_cast<shapes::Mesh*>(shape));
    break;
  }
  default:
    break;

  }

  // Need to cleanup pointer
  if (shape != NULL)
    delete shape;


  if (cg)
    cg->computeLocalAABB();

  return cg;
}

/**
 * @function constructShape
 * @brief Get shapes:Shape pointer from a urdf::Geometry pointer
 */
shapes::Shape* constructShape(const urdf::Geometry *geom)
{

  shapes::Shape *result = NULL;

  if(!geom);
    return result;

  switch (geom->type)
  {
  case urdf::Geometry::SPHERE:
    result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
    break;
  case urdf::Geometry::BOX:
  {
    urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
    result = new shapes::Box(dim.x, dim.y, dim.z);
  }
  break;
  case urdf::Geometry::CYLINDER:
    result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius,
                                  dynamic_cast<const urdf::Cylinder*>(geom)->length);
    break;
  case urdf::Geometry::MESH:
  {
    const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
    if (!mesh->filename.empty())
    {
      Eigen::Vector3d scaling(mesh->scale.x, mesh->scale.y, mesh->scale.z);
      if (scaling.norm() == 0)
      {
        scaling << 1, 1, 1;
      }
      result = shapes::createMeshFromResource(mesh->filename, scaling);
    } // if mesh->filename empty
    else
      RCLCPP_WARN(rclcpp::get_logger("robot_entity"), "Empty mesh filename");
  } // case urdf::Geometry::MESH

  break;
  default:
    RCLCPP_ERROR(rclcpp::get_logger("robot_entity"), "Unknown geometry type: %d", (int)geom->type);
    break;
  }

  return result;
}

/**
 * @function meshToBvh
 */
fcl::BVHModel<fcl::OBBRSSd>* meshToBvh(shapes::Mesh* _mesh)
{
  fcl::BVHModel<fcl::OBBRSSd>* bvh = new fcl::BVHModel<fcl::OBBRSSd>();
  if (_mesh->vertex_count > 0 && _mesh->triangle_count > 0)
  {
    std::vector<fcl::Triangle> tri_indices(_mesh->triangle_count);
    for (unsigned int i = 0; i < _mesh->triangle_count; i++)
      tri_indices[i] =
        fcl::Triangle(_mesh->triangles[3 * i], _mesh->triangles[3 * i + 1], _mesh->triangles[3 * i + 2]);

    std::vector<fcl::Vector3d> points(_mesh->vertex_count);
    for (unsigned int i = 0; i < _mesh->vertex_count; i++)
      points[i] = fcl::Vector3d(_mesh->vertices[3 * i], _mesh->vertices[3 * i + 1], _mesh->vertices[3 * i + 2]);

    bvh->beginModel();
    bvh->addSubModel(points, tri_indices);
    bvh->endModel();
  }

  return bvh;
}


bool isValidPose(const geometry_msgs::msg::PoseStamped &_pose)
{
  geometry_msgs::msg::Quaternion q = _pose.pose.orientation;
  Eigen::Quaternion qe(q.w, q.x, q.y, q.z);

  if (qe.norm() == 0.0)
    return false;

  if (_pose.header.frame_id.empty())
    return false;

  return true;
}

sensor_msgs::msg::JointState getJointStateFromMap(const std::map<std::string, double> &_js)
{
  sensor_msgs::msg::JointState msg;
  for (auto ji : _js)
  {
    msg.name.push_back(ji.first);
    msg.position.push_back(ji.second);
  }
  return msg;
}
