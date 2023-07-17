#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robot_unit/fast_base_collision_object.h>
#include <robot_unit/robot_entity.h>
#include <robot_unit/fast_helpers.h>

#include <robot_unit/fast_allowed_collision_matrix.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace robot_unit
{

/**
 * @class RobotCollisionObject
 */
class RobotCollisionObject : public BaseCollisionObject
{
public:
  RobotCollisionObject();
  ~RobotCollisionObject();
  
  bool init(const std::string &_ref_frame,
            const std::string &_robot_name,
            const std::string &_urdf_string,
            const std::string &_srdf_string,
            const double &_padding = 0.0);

  bool update(const sensor_msgs::msg::JointState &_js);
  bool updateReferenceTransform(const geometry_msgs::msg::PoseStamped &_pose);
  bool selfCollide();

  bool getRobotRootLink(std::string &_root_link_name);
  void getRobotInfo(std::string &_name,
                    std::string &_robot_description,
                    std::string &_tf_prefix);
  bool setJointState(const std::map<std::string, double> &_joints,
                     sensor_msgs::msg::JointState &_js);
  void getRobotLatestState(sensor_msgs::msg::JointState &_js,
                           geometry_msgs::msg::PoseStamped &_pose);

  bool updateLastJointVals(const sensor_msgs::msg::JointState &_js);

  bool getCollisionMarkers(const std::string &_link,
                          visualization_msgs::msg::MarkerArray &_msg);
  bool getCollisionMarkers(const std::string &_link, 
                           const geometry_msgs::msg::PoseStamped &_pose,
                          visualization_msgs::msg::MarkerArray &_msg);

protected:
  RobotEntity re_;
  std::shared_ptr<AllowedCollisionMatrix> acm_;

  std::string robot_name_;
  std::string robot_description_;
  std::string robot_description_semantic_;
  std::string tf_prefix_;


  // Reference frame Tf
  KDL::Frame Tf_ref_;
  std::map<std::string, double> last_joint_vals_;

  bool initialized_;
};


typedef std::shared_ptr<RobotCollisionObject> RobotCollisionObjectPtr;


} // namespace robot_unit
