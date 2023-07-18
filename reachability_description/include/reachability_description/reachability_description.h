#ifndef __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__
#define __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__

#include <rclcpp/rclcpp.hpp>
#include <trac_ik/trac_ik.hpp>

#include <urdf/model.h>
#include <srdfdom/model.h>

#include <robot_unit/fast_robot_collision_object.h>
#include <Eigen/Geometry>

#include <reachability_description/reach_data.h>

namespace reachability_description
{

/**
 * @class ReachabilityDescription
 */
class ReachabilityDescription
{
    public:
    ReachabilityDescription(const rclcpp::Node::SharedPtr &_nh);
    ~ReachabilityDescription();

    bool initialize(const double &_max_ik_time = 0.001, 
                    const double &_eps = 1e-5, 
                    const TRAC_IK::SolveType &_ik_type = TRAC_IK::SolveType::Distance);
    bool quickTest();
    bool generateDescription();

    protected:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_reach_;

  std::shared_ptr<RobotEntity> re_;
  std::shared_ptr<robot_unit::RobotCollisionObject> rco_;

  std::shared_ptr<ReachGraph> reach_graph_;

};

} // namespace reachability_description

#endif //