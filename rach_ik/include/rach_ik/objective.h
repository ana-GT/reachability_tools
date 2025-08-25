#pragma once

#include <kdl/chainfksolverpos_recursive.hpp>
#include <Eigen/Geometry>
#include <memory>

struct ObjectiveData {
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    Eigen::Vector3d goal_pos;
    Eigen::Quaterniond goal_rot;
};

void calculateEEDiff(const std::vector<double> &x,
                     double &_dlin, double &_drot,  
                     void *objective_data );

