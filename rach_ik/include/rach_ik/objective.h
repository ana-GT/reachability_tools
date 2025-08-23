#pragma once

#include <kdl/chainfksolverpos_recursive.hpp>
#include <Eigen/Geometry>
#include <memory>

struct ObjectiveData {
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    Eigen::Vector3d goal_pos;
    Eigen::Quaterniond goal_rot;
};

void calculateError(const std::vector<double> &x, double &_error, void *objective_data );
double cost_function(const std::vector<double> &x, std::vector<double> &grad, void *objective_data);