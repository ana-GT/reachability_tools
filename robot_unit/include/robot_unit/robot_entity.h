#pragma once

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_unit/treefksolverpos_recursive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <reachability_msgs/msg/chain_info.hpp>

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
  bool getChainInfo(const std::string &_chain_name, 
                    reachability_msgs::msg::ChainInfo &_chain_info);
  bool getKDLChain(const std::string &_root_link, const std::string &_tip_link,
                KDL::Chain &_chain);
  bool getJointLimits(const std::vector<std::string> &_joint_names,
                      std::vector<std::pair<double, double>> &_joint_limits);
  bool getChainGroupState(const std::string &_chain_group,
                          const std::string &_state_name,
                          KDL::JntArray &_state);

protected:

  bool getIndices();
  void printInfo();
  bool getMimicInfo();


  std::shared_ptr<urdf::Model> urdf_;
  std::shared_ptr<srdf::Model> srdf_model_;
  KDL::Tree tree_;
  std::shared_ptr<robot_unit::TreeFkSolverPos_recursive> fk_;

  std::vector<std::string> joint_names_;
  std::map<std::string, double> joint_lower_lim_;
  std::map<std::string, double> joint_upper_lim_;
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
