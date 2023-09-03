/**
 * @file fast_robot_collision_object.cpp
 */
#include <robot_unit/fast_robot_collision_object.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>

using namespace robot_unit;

RobotCollisionObject::RobotCollisionObject() :
  BaseCollisionObject()
{
  Tf_ref_ = KDL::Frame::Identity();
  initialized_ = false;
}

RobotCollisionObject::~RobotCollisionObject()
{
  printf("RobotCollision Object destructor... \n");
}

bool RobotCollisionObject::setJointState(const std::map<std::string, double> &_joints,
    sensor_msgs::msg::JointState &_js)
{
  // Update with the latest joints recorded
  std::map<std::string, double> last_joints = last_joint_vals_;

  for (auto ji : _joints)
  {
    if (last_joints.find(ji.first) == last_joints.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
       "Joint %s not found in record, ignoring!", ji.first.c_str());
      return false;
    }

    last_joints[ ji.first ] = ji.second;
  }

  // Fill
  _js.name.clear();
  _js.position.clear();

  for (auto ji : last_joints)
  {
    _js.name.push_back(ji.first);
    _js.position.push_back(ji.second);
  }

  return true;
}

bool RobotCollisionObject::init(const std::string &_ref_frame,
                                const std::string &_robot_name,
                                const std::string &_urdf_string,
                                const std::string &_srdf_string, const double &_padding)
{
  init_(_ref_frame);
  robot_name_ = _robot_name;
  robot_description_ = _urdf_string;
  robot_description_semantic_ = _srdf_string;
  tf_prefix_ = robot_name_;

  if (!re_.init(_urdf_string, _srdf_string))
    return false;

  // Init Collision Matrix
  std::vector<std::string> links;
  links = re_.getLinkNames();
  acm_.reset(new AllowedCollisionMatrix(links, false));

  // Ignore links adjacent
  std::map<std::string, std::string> link_parents;
  link_parents = re_.getLinkParentNames();
  for (auto li : link_parents)
    acm_->setEntry(li.first, li.second, true);

  // Ignore SRDF disabled collisions, if available
  std::vector<std::pair<std::string, std::string>> disabled_srdf;
  disabled_srdf = re_.getSrdfDisabledCollPairs();
  for (auto ds : disabled_srdf)
    acm_->setEntry(ds.first, ds.second, true);

  // Init FLCObjects
  std::vector<std::pair<std::string, LinkCollInfo > > link_colls;
  link_colls = re_.getLinkCollisions();
  
  for (int i = 0; i < link_colls.size(); ++i)
  {
    fcl::CollisionGeometryd* cg = urdfCollisionToFcl(link_colls[i].second.coll_,
                                  _padding, 1.0);

    if (!cg)
      return false;

    fcl::Transform3d local_tf;
    local_tf = urdfCollisionToFclTf(link_colls[i].second.coll_);

    std::string id = link_colls[i].first.c_str(); // link name
    int shape_index = link_colls[i].second.local_shape_index_;
    FCLGeometryConstPtr g(new FCLGeometry(cg, id, shape_index, local_tf));

    fcl::CollisionObjectd* fcl_obj = new fcl::CollisionObjectd(g->collision_geometry_,
        local_tf);

    this->addShape(fcl_obj, g);
  }

  // Set last_joint_vals with joint names
  last_joint_vals_.clear();
  for (auto ji : re_.getJointNames())
    last_joint_vals_[ji] = 0.0;

  // Initialized correctly
  initialized_ = true;

  return initialized_;
}

/**
 * @function updateLastJointVals
 */
bool RobotCollisionObject::updateLastJointVals(const sensor_msgs::msg::JointState &_js)
{
  if (_js.name.size() != _js.position.size())
    return false;

  for (int i = 0; i < _js.name.size(); ++i)
  {
    if (last_joint_vals_.find(_js.name[i]) == last_joint_vals_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "updateLastJointVals: Found unknown joint name! : %s . Ignoring", _js.name[i].c_str());
      return false;
    }
    last_joint_vals_[_js.name[i]] = _js.position[i];
  }
  return true;
}

bool RobotCollisionObject::update(const sensor_msgs::msg::JointState &_js)
{
  // Store latest joint state, if not empty
  updateLastJointVals(_js);

  std::map<std::string, KDL::Frame> Tfxs;
  re_.calculateTransforms(getJointStateFromMap(last_joint_vals_), Tfxs);

  for (int i = 0; i < object_.collision_objects_.size(); ++i)
  {
    std::string link = object_.collision_geometry_[i]->collision_geometry_data_->id_;

    Eigen::Isometry3d Tfi;
    tf2::transformKDLToEigen(Tfxs[link], Tfi);
    Tfi = Tfi * object_.collision_geometry_[i]->collision_geometry_data_->local_tf_;

    object_.collision_objects_[i]->setTransform(Tfi);
    object_.collision_objects_[i]->computeAABB();
  }

  manager_->update();
  return true;
}

bool RobotCollisionObject::updateReferenceTransform(const geometry_msgs::msg::PoseStamped &_pose)
{
  // Note: TODO Check that it is w.r.t. reference_frame_
  KDL::Frame Tfx;
  tf2::fromMsg(_pose.pose, Tfx);

  re_.updateTfRefRoot(Tfx);
  Tf_ref_ = Tfx;
  return true;
}


bool RobotCollisionObject::selfCollide()
{
  SimpleCollisionData cd;
  cd.acm_ = acm_.get();
  //cd.request.enable_contact = false;
  manager_->collide(&cd, SimpleCollisionFunction);
  return cd.result.isCollision();

}

bool RobotCollisionObject::getRobotRootLink(std::string &_root_link_name)
{
  _root_link_name = re_.getRootLinkName();
  return true;
}

void RobotCollisionObject::getRobotInfo(std::string &_name,
                                        std::string &_robot_description,
                                        std::string &_tf_prefix)
{
  _name = robot_name_;
  _robot_description = robot_description_;
  _tf_prefix = tf_prefix_;
}


/**
 * @function getRobotLatestState
 * @brief Get the latest joint and pose of the robot
 */
void RobotCollisionObject::getRobotLatestState(sensor_msgs::msg::JointState &_js,
    geometry_msgs::msg::PoseStamped &_pose)
{
  // Fill js
  _js = getJointStateFromMap(last_joint_vals_);

  // Fill Tf_ref
  _pose.pose = tf2::toMsg(Tf_ref_);
  _pose.header.frame_id = reference_frame_;

}

bool RobotCollisionObject::getCollisionMarkers(const std::string &_link,
                                               visualization_msgs::msg::MarkerArray &_msg)
{
  return re_.getCollisionMarkers(_link, _msg);
}

bool RobotCollisionObject::getCollisionMarkers(const std::string &_link, 
                                               const geometry_msgs::msg::PoseStamped &_pose,
                                               visualization_msgs::msg::MarkerArray &_msg)
{
  if(!re_.getCollisionMarkers(_link, _msg))
    return false;

  for (int i = 0; i < _msg.markers.size(); ++i)
    _msg.markers[i].pose = _pose.pose;

  return true;
}
