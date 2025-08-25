#pragma once

#include <vector>
#include <memory>
#include <kdl/chainfksolverpos_recursive.hpp>


struct ConstraintData {
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    int elbow_index;
    int wrist_index;
};


double elbow_wrist_cosine(const std::vector<double> &_x, void *_data);
double constraint_elbow_down(const std::vector<double> &x, std::vector<double> &_grad, void *_data);
