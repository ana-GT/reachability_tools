#pragma once

#include <rach_ik/optimize.h>

/**
 * @class MobileHeuristic
 */
namespace rach_ik_plugins
{

class MobileHeuristicOptimizer : public RachOptimizer {

public :
    MobileHeuristicOptimizer();
    virtual bool init_();
    virtual bool getConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_goal_pose,
                const sensor_msgs::msg::JointState &_js,
                sensor_msgs::msg::JointState &_sol);

    virtual bool getMobileConfiguration( const std::string &_group,
                const geometry_msgs::msg::PoseStamped &_goal_pose,
                const sensor_msgs::msg::JointState &_init_js,
                const geometry_msgs::msg::PoseStamped &_init_base_pose,
                sensor_msgs::msg::JointState &_sol_arm_config,
                geometry_msgs::msg::PoseStamped &_sol_base_pose);

protected:

    void getCurrentBasePlacement(const std::string &_ref_frame, double &_x0, double &_y0, double &_yaw0);

    // Constraints
    std::string elbow_link_;
    std::string wrist_link_;
    std::vector<std::string> reference_poses_;
};

} // namespace rach_ik_plugins
