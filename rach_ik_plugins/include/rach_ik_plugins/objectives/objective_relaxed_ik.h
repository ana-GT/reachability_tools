#pragma once

#include <rach_ik/objective.h>

double cost_relaxed_ik_function(const std::vector<double> &x, std::vector<double> &grad, void *objective_data);


//===============================
// Basic Loss functions
//===============================

double gaussianLoss(const double &_x, const double &_g, 
                    const double &_c);
double wallLoss(const double &_x, const double &_l, const double &_u,
                const double &_a1, const double &_b,
                const int &_n);
double polynomialLoss(const double &_x, const double &_g, 
                      const double &_a2, const int &_m);

//===============================
// Parametric loss functions
//===============================

double specificGoalLoss(const double &_x, const double &_g,
                        const double &_c, const double &_a2, const int &_m);

double rangedGoalEquallyValidLoss(const double &_x, const double &_l, const double &_u,
                                  const double &_a1, const double &_a2, const int &_m,
                                  const double &_b, const int &_n);

double rangedGoalPreferredLoss(const double &_x, const double &_l, const double &_u,
                               const double &_g, const double &_c,
                               const double &_a1, const double &_a2, const int &_m,
                               const double &_b, const int &_n);                                  
