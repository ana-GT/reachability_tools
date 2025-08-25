#pragma once

#include <rach_ik/optimize.h>

/**
 * @class RachOptimizer
 */
namespace rach_ik_plugins
{

class LowElbowOptimizer : public RachOptimizer {

public :
    LowElbowOptimizer();
    virtual bool init_();
    virtual bool getConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_pose,
                const sensor_msgs::msg::JointState &_js,
                sensor_msgs::msg::JointState &_sol);

protected:

    // Constraints
    std::string elbow_link_;
    std::string wrist_link_;
    std::vector<std::string> reference_poses_;
};

} // namespace rach_ik_plugins
