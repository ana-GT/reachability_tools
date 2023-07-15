#ifndef __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__
#define __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__

#include <rclcpp/rclcpp.hpp>
#include <trac_ik/trac_ik.hpp>

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

    bool initialize(const std::string &_chain_group,
                    const double &_max_ik_time = 0.005, 
                    const double &_eps = 1e-5, 
                    const TRAC_IK::SolveType &_ik_type = TRAC_IK::SolveType::Distance);

    protected:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_;

};

} // namespace reachability_description

#endif //