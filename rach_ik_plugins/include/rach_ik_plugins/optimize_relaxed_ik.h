#pragma once

#include <rach_ik/optimize.h>

/**
 * @class RachOptimizer
 */
namespace rach_ik_plugins
{

class RelaxedIKOptimizer : public RachOptimizer {

public :
    RelaxedIKOptimizer();
    virtual bool init_();
    virtual bool getConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_pose,
                const sensor_msgs::msg::JointState &_js,
                const bool &_mobile,
                sensor_msgs::msg::JointState &_sol,
                geometry_msgs::msg::PoseStamped &_base_pose);

protected:

    // Constraints
    std::vector<std::string> reference_poses_;
};

} // namespace rach_ik_plugins
