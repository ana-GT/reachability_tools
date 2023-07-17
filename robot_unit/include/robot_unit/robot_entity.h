#pragma once

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_unit/treefksolverpos_recursive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/**
 * @struct JointMimicInfo
 */
struct JointMimicInfo
{
  JointMimicInfo()
  {
    mult = 1.0;
    offset = 0.0;
  }
  double mult;
  double offset;
  std::string joint_name;
};

struct LinkCollInfo
{
  urdf::CollisionSharedPtr coll_;
  int local_shape_index_; // Index w.r.t. its parent link
};


/**
 * @class RobotEntity
 */
class RobotEntity
{
public:
  RobotEntity();
  ~RobotEntity();

  virtual bool init(const std::string &_urdf_string, 
                    const std::string &_srdf_string);

  bool updateTfRefRoot(const KDL::Frame &_Tf_ref_root);
  bool calculateTransforms(const sensor_msgs::msg::JointState &_js,
                           std::map<std::string, KDL::Frame> &_tfs);
  bool getJointsOrdered(const sensor_msgs::msg::JointState &_js,
                        KDL::JntArray &_q);
  std::vector<std::pair<std::string, LinkCollInfo> > getLinkCollisions();

  std::vector<std::string> getLinkNames();
  std::vector<std::string> getJointNames();
  std::map<std::string, std::string> getLinkParentNames();
  std::string getRootLinkName();
  std::vector<std::pair<std::string, std::string>>  getSrdfDisabledCollPairs();
  bool getCollisionMarkers(const std::string &_link, 
                           visualization_msgs::msg::MarkerArray &_markers);

protected:

  bool getIndices();
  void printInfo();
  bool getMimicInfo();


  std::shared_ptr<urdf::Model> urdf_;
  std::shared_ptr<srdf::Model> srdf_model_;
  KDL::Tree tree_;
  std::shared_ptr<robot_unit::TreeFkSolverPos_recursive> fk_;

  std::vector<std::string> joint_names_;
  std::map<std::string, int> joint_indices_;
  std::vector<std::string> link_names_;
  std::map<std::string, std::string> link_parent_names_;
  std::map<std::string, std::string> joint_child_link_names_;
  std::string root_link_name_;

  int num_links_; // number of segments
  int num_joints_;

  std::map<std::string, JointMimicInfo> mimic_info_;
  std::vector<std::pair<std::string, LinkCollInfo> > link_collisions_;

  // Srdf
  std::vector<std::pair<std::string, std::string>> srdf_disabled_coll_pairs_;

};
