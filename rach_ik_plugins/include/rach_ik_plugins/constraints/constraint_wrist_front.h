#pragma once

#include <vector>
#include <memory>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace constraint_wrist {

struct ConstraintData {
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    int elbow_index;
    int wrist_index;
};


double wrist_front(const std::vector<double> &_x, void *_data);
double constraint_wrist_front(const std::vector<double> &x, std::vector<double> &_grad, void *_data);

}
