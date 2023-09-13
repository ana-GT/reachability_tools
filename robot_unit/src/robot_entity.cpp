/**
 * @file robot_entity.cpp
 * @brief
 * @date 2023/07/16
 */
#include <robot_unit/robot_entity.h>

/**
 * @brief Constructor 
 */
RobotEntity::RobotEntity()
{

}

/**
 * @brief Destructor 
 */
RobotEntity::~RobotEntity()
{
}


bool RobotEntity::init(const std::string &_urdf_string,
                       const std::string &_srdf_string)
{
  urdf_.reset(new urdf::Model());
  srdf_model_.reset(new srdf::Model());

  if (!urdf_->initString(_urdf_string))
    return false;

  if (srdf_model_->initString(*urdf_, _srdf_string))
  {
      std::vector<srdf::Model::CollisionPair> ignored = srdf_model_->getDisabledCollisionPairs();
      for (auto ign : ignored)
        srdf_disabled_coll_pairs_.push_back(std::pair<std::string, std::string>(ign.link1_, ign.link2_));
  } else
    return false;


  // Init KDL Tree
  if(!kdl_parser::treeFromUrdfModel(*urdf_, tree_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("debug"), "KDL TREE START FROM URDF MODEL - RE");
    return false;
  }
  // Init FK
  fk_.reset(new robot_unit::TreeFkSolverPos_recursive(tree_));

  // Get indices for joints/links
  if (!getIndices())
    return false;

  // Get mimic joint information
  if (!getMimicInfo())
    return false;

  // Get collisions
  link_collisions_.clear();
  for (auto li : urdf_->links_)
  {
    //printf("Link %s has %lu collision bodies \n", li.first.c_str(),
    //     li.second->collision_array.size());
    int idx = 0;
    for (auto lci : li.second->collision_array)
    {
      LinkCollInfo lcio;
      lcio.coll_ = lci;
      lcio.local_shape_index_ = idx;
      link_collisions_.push_back(std::make_pair(li.first, lcio));
      idx++;
    }
  }

  // Get root
  root_link_name_ = tree_.getRootSegment()->second.segment.getName();

  return true;
}

/**
 * @function getMimicInfo 
 */
bool RobotEntity::getMimicInfo()
{
  for (auto ji : joint_names_)
  {
    if (!urdf_->getJoint(ji))
      return false;

    auto jn = urdf_->getJoint(ji);
    if (jn->mimic)
    {
      printf("Joint %s is mimic joint \n", ji.c_str());
      JointMimicInfo jmi;
      jmi.mult = jn->mimic->multiplier;
      jmi.offset = jn->mimic->offset;
      jmi.joint_name = jn->mimic->joint_name;
      mimic_info_[ji] = jmi;
    }

  }

  return true;
}

bool isValidJointType(KDL::Joint::JointType joint_type)
{
 if(joint_type == KDL::Joint::JointType::RotAxis ||
    joint_type == KDL::Joint::JointType::RotX ||
    joint_type == KDL::Joint::JointType::RotY ||
    joint_type == KDL::Joint::JointType::RotZ ||
    joint_type == KDL::Joint::JointType::TransAxis ||
    joint_type == KDL::Joint::JointType::TransX ||
    joint_type == KDL::Joint::JointType::TransY ||
    joint_type == KDL::Joint::JointType::TransZ )
    return true;

  return false;
}

/**
 * @function getChainInfo 
 */
bool RobotEntity::getChainInfo(const std::string &_chain_name, 
                               reachability_msgs::msg::ChainInfo &_chain_info)
{
  std::vector<srdf::Model::Group> groups = srdf_model_->getGroups();
  bool found_chain = false;

  for(int i = 0; i < groups.size(); ++i)
  {
    if(groups[i].name_ == _chain_name)
    {
        if(groups[i].chains_.size() == 1)
        {
         _chain_info.group = _chain_name;
         _chain_info.root_link = groups[i].chains_[0].first;
         _chain_info.tip_link = groups[i].chains_[0].second;

         KDL::Chain chain;
         tree_.getChain(_chain_info.root_link, _chain_info.tip_link, chain);
         for(auto si : chain.segments)
         {
          if( isValidJointType(si.getJoint().getType()) )
            _chain_info.joint_names.push_back(si.getJoint().getName());
         }

         _chain_info.num_joints = chain.getNrOfJoints(); // Should be equal to joint_names.size()

         found_chain = true;
         break;
        }
    }
  }

  return found_chain;
}

/**
 * @function getKDLChain 
 */
bool RobotEntity::getKDLChain(const std::string &_root_link, const std::string &_tip_link,
                              KDL::Chain &_chain)
{
  return tree_.getChain(_root_link, _tip_link, _chain);  
}


/**
 * @function getIndices
 */
bool RobotEntity::getIndices()
{
  // segments seem not to count the first link, but you can access it anyway
  num_links_ = tree_.getNrOfSegments() + 1;
  num_joints_ = tree_.getNrOfJoints();

  // 1. Get joints ordered
  joint_names_.clear();
  joint_names_.resize(num_joints_);
  joint_indices_.clear();

  for (auto si : tree_.getSegments())
  {
    if (si.second.q_nr < num_joints_ &&
        si.second.segment.getJoint().getType() != KDL::Joint::JointType::None)
    {
      auto jn = si.second.segment.getJoint().getName();
      joint_names_[si.second.q_nr] = jn;
      joint_indices_[jn] = si.second.q_nr;
    }
  }


  // 2. Get links ordered
  fk_->getSegmentsRecursiveOrder(link_names_);

  // 3. Get link parent
  link_parent_names_.clear();
  for (auto li : link_names_)
  {
    if (urdf_->getLink(li))
      if (urdf_->getLink(li)->getParent())
        link_parent_names_[li] = urdf_->getLink(li)->getParent()->name;
  }

  // 4. Get joint's child link names
  joint_child_link_names_.clear();

  for (auto ji : joint_names_)
  {
    if (urdf_->getJoint(ji))
    {
      joint_child_link_names_[ji] = urdf_->getJoint(ji)->child_link_name;
 
      if(urdf_->getJoint(ji)->type == urdf::Joint::PRISMATIC ||
         urdf_->getJoint(ji)->type == urdf::Joint::REVOLUTE)
      {     
        joint_lower_lim_[ji] = urdf_->getJoint(ji)->limits->lower;
        joint_upper_lim_[ji] = urdf_->getJoint(ji)->limits->upper;
      } 
    }
  }

  return true;
}

bool RobotEntity::calculateTransforms(const sensor_msgs::msg::JointState &_js,
                                      std::map<std::string, KDL::Frame> &_tfs)
{
  // 1. Order joints, if not ordered already
  KDL::JntArray q;
  if (!getJointsOrdered(_js, q))
  {
    printf("Returning false calculate transform! \n");
    return false;
  }
  // 2. Get transforms
  int res = fk_->JntToCart(q, _tfs);

  return true;
}

bool RobotEntity::getJointsOrdered(const sensor_msgs::msg::JointState &_js,
                                   KDL::JntArray &_q)
{
  if (_js.position.size() < num_joints_ - mimic_info_.size())
  {
    printf("Joints of size: %lu , whereas num joints: %d mimic: %lu \n",
           _js.position.size(), num_joints_, mimic_info_.size());
    return false;
  }

  _q.data.resize(num_joints_);

  int filled = 0;
  for (int i = 0; i < _js.name.size(); ++i)
  {
    if (joint_indices_.find(_js.name[i]) == joint_indices_.end())
      return false;
    else
    {
      _q(joint_indices_[_js.name[i]]) = _js.position[i];
      filled++;
    }
  }

  if (filled == num_joints_)
    return true;
  else
    return false;
}

bool RobotEntity::updateTfRefRoot(const KDL::Frame &_Tf_ref_root)
{
  fk_->updateTfRefRoot(_Tf_ref_root);
  return true;
}

std::vector<std::pair<std::string, LinkCollInfo> > RobotEntity::getLinkCollisions()
{
  return link_collisions_;
}

void RobotEntity::printInfo()
{

}

std::vector<std::string> RobotEntity::getLinkNames()
{
  return link_names_;
}

std::vector<std::string> RobotEntity::getJointNames()
{
  return joint_names_;
}

std::map<std::string, std::string> RobotEntity::getLinkParentNames()
{
  return link_parent_names_;
}

std::string RobotEntity::getRootLinkName()
{
  return root_link_name_;
}

std::vector<std::pair<std::string, std::string>>  RobotEntity::getSrdfDisabledCollPairs()
{
  return srdf_disabled_coll_pairs_;
}

bool RobotEntity::getJointLimits(const std::vector<std::string> &_joint_names,
                                 std::vector<std::pair<double, double> > &_joint_limits)
{
  for(auto ji : _joint_names)
  {
    if(joint_lower_lim_.find(ji) == joint_lower_lim_.end() ||
       joint_upper_lim_.find(ji) == joint_upper_lim_.end() )
      return false;
    
    _joint_limits.push_back(std::pair<double, double>(joint_lower_lim_[ji], joint_upper_lim_[ji]));
  }

  return true;
}


bool RobotEntity::getCollisionMarkers(const std::string &_link, 
                                      visualization_msgs::msg::MarkerArray &_markers)
{

  urdf::LinkConstSharedPtr link = urdf_->getLink(_link);
  if (!link)
    return false;

  std::vector<urdf::CollisionSharedPtr> collisions;
  urdf::CollisionSharedPtr collision = link->collision;

  if (link->collision_array.size() > 0)
    collisions = link->collision_array;
  else if (collision)
    collisions.push_back(collision);

  for (int i = 0; i < collisions.size(); ++i)
  {
    visualization_msgs::msg::Marker mi;
    urdf::GeometrySharedPtr geom = collisions[i]->geometry;
    if (!geom)
      return false;

    if (geom->type == urdf::Geometry::MESH)
    {
      urdf::Mesh *m = dynamic_cast<urdf::Mesh*>(geom.get());
      mi.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      mi.mesh_resource = m->filename;
      mi.scale.x = m->scale.x;
      mi.scale.y = m->scale.y;
      mi.scale.z = m->scale.z;
      _markers.markers.push_back(mi);
    }
    else if (geom->type == urdf::Geometry::SPHERE)
    {
      urdf::Sphere *m = dynamic_cast<urdf::Sphere*>(geom.get());
      mi.type = visualization_msgs::msg::Marker::SPHERE;
      mi.scale.x = m->radius;
      mi.scale.y = m->radius;
      mi.scale.z = m->radius;

      _markers.markers.push_back(mi);
    }
    else if (geom->type == urdf::Geometry::CYLINDER)
    {
      urdf::Cylinder *m = dynamic_cast<urdf::Cylinder*>(geom.get());
      mi.type = visualization_msgs::msg::Marker::CYLINDER;
      mi.scale.x = m->radius * 2.0;
      mi.scale.y = m->radius * 2.0;
      mi.scale.z = m->length;

      _markers.markers.push_back(mi);
    }
    else if (geom->type == urdf::Geometry::BOX)
    {
      urdf::Box *m = dynamic_cast<urdf::Box*>(geom.get());
      mi.type = visualization_msgs::msg::Marker::CUBE;
      mi.scale.x = m->dim.x;
      mi.scale.y = m->dim.y;
      mi.scale.z = m->dim.z;

      _markers.markers.push_back(mi);
    }
  }

  return true;
}

/**
 * @function getChainGroupState 
 */
bool RobotEntity::getChainGroupState(const std::string &_chain_group,
                                     const std::string &_state_name,
                                     KDL::JntArray &_state)
{

  reachability_msgs::msg::ChainInfo ci;
  if(!this->getChainInfo(_chain_group, ci))
    return false;

  std::vector<srdf::Model::GroupState> gs = srdf_model_->getGroupStates();
  std::map<std::string, double> sm;
  for(auto gi : srdf_model_->getGroupStates())
  {
    if( gi.group_ == _chain_group)
    {
      if(gi.name_ == _state_name)
      {
        for(auto ai : gi.joint_values_)
        {
          if(ai.second.size() == 0)
            return false;

          sm[ai.first] = ai.second[0];
        } // for

        break;
      } // if gi.name

    } // if gi.group
  } // for gi

  if(sm.size() != ci.joint_names.size())
    return false;

  _state.data.resize(ci.joint_names.size());
  for(int i = 0; i < ci.joint_names.size(); ++i)
    _state(i) = sm[ci.joint_names[i]];
  

  return true;
}