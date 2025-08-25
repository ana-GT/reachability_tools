#pragma once

#include <rach_ik/objective.h>
#include <rach_ik/conversion_utils.h>

struct MobileObjectiveData {
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    Eigen::Vector3d goal_pos;
    Eigen::Quaterniond goal_rot;
    Eigen::Isometry3d Tf_base_root;
};

// DOFs + 3 (x, y, theta)
double cost_mobile_ee_diff_function(const std::vector<double> &x, std::vector<double> &grad, void *objective_data);
