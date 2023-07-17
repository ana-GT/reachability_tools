#ifndef __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__
#define __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__

#include <rclcpp/rclcpp.hpp>
#include <trac_ik/trac_ik.hpp>

#include <urdf/model.h>
#include <srdfdom/model.h>

#include <robot_unit/fast_robot_collision_object.h>

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

    bool initialize(const double &_max_ik_time = 0.005, 
                    const double &_eps = 1e-5, 
                    const TRAC_IK::SolveType &_ik_type = TRAC_IK::SolveType::Distance);

    bool generateDescription();

    protected:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_;

  std::shared_ptr<urdf::Model> urdf_;
  std::shared_ptr<srdf::Model> srdf_model_;

  std::shared_ptr<robot_unit::RobotCollisionObject> rco_;


};

} // namespace reachability_description

#endif //