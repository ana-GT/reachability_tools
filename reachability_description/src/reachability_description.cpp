#include <reachability_description/reachability_description.h>

namespace reachability_description
{

/**
 * @function ReachabilityDescription
 * @brief Constructor 
 */
ReachabilityDescription::ReachabilityDescription(const rclcpp::Node::SharedPtr &_nh) :
nh_(_nh)
{

}

/**
 * @function ReachabilityDescription
 * @brief Destructor
 */
ReachabilityDescription::~ReachabilityDescription()
{

}

/**
 * @function initialize
 * @brief Well, initialize
 */
bool ReachabilityDescription::initialize(const std::string &_chain_group,
                                         const double &_max_time, 
                                         const double &_eps, 
                                         const TRAC_IK::SolveType &_ik_type)
{
    std::string base_link, tip_link;

    ik_solver_.reset( new TRAC_IK::TRAC_IK(nh_, base_link, tip_link, "robot_description", _max_time, _eps, _ik_type));

    return true;
}


}  // namespace reachability_description