#pragma once

#include <rach_ik/objective.h>

double cost_ee_diff_function(const std::vector<double> &x, std::vector<double> &grad, void *objective_data);



