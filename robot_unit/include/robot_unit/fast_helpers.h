/**
 * @file fast_helpers.h
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>

#include <urdf/model.h>

#include <geometric_shapes/mesh_operations.h>

fcl::Transform3d urdfCollisionToFclTf(const urdf::CollisionSharedPtr &_coll);
fcl::CollisionGeometryd* urdfCollisionToFcl(const urdf::CollisionSharedPtr &_coll,
    double _padding, double _scale);
shapes::Shape* constructShape(const urdf::Geometry *geom);
fcl::BVHModel<fcl::OBBRSSd>* meshToBvh(shapes::Mesh* _mesh);

bool isValidPose(const geometry_msgs::msg::PoseStamped &_pose);
sensor_msgs::msg::JointState getJointStateFromMap(const std::map<std::string, double> &_js);
