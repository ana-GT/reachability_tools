
#include <rach_ik_plugins/objectives/objective_relaxed_ik.h>

#include <cfloat>

double cost_relaxed_ik_function(const std::vector<double> &x, std::vector<double> &grad, void *objective_data)
{
  double dlin, drot;
  calculateEEDiff(x, dlin, drot, objective_data);

  double error;
  double c, a2, m;
  c = 0.1; a2 = 10.0; m = 2;
  error = specificGoalLoss(dlin, 0.0, c, a2, m) + specificGoalLoss(drot, 0.0, c, a2, m);

  // Grad
  std::vector<double> vals(x);

  double jump = FLT_EPSILON;

  if (!grad.empty())
  {
    double v1;
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;

      calculateEEDiff(vals, dlin, drot, objective_data);
      v1 = specificGoalLoss(dlin, 0.0, c, a2, m)  + specificGoalLoss(drot, 0.0, c, a2, m);

      vals[i] = original;
      grad[i] = (v1 - error) / (2 * jump);
    }
  }

  return error;
}

////////////////////////////////////
// BASIC FUNCTIONS
////////////////////////////////////
double gaussianLoss(const double &_x, const double &_g, 
                    const double &_c)
{
  return -1.0*exp( (-1*pow( (_x - _g), 2)) / (2*pow(_c, 2)) );
}

double wallLoss(const double &_x, const double &_l, const double &_u,
                const double &_a1, const double &_b,
                const int &_n) 
{
   // xp in [-1, 1]
   double xp = (2*_x - _l - _u)/(_u - _l);

   return _a1 * (1.0 - exp( -1.0*pow(xp, _n) / pow(_b, _n) ) ); 
}

double polynomialLoss(const double &_x, const double &_g, 
                      const double &_a2, const int &_m)
{
  return _a2 * pow( (_x - _g), _m);
}

/////////////////////////////////////////
// PARAMETRIC LOSS FUNCTIONS
/////////////////////////////////////////

double specificGoalLoss(const double &_x, const double &_g,
                        const double &_c, const double &_a2, const int &_m)
{
  return gaussianLoss(_x, _g, _c) + polynomialLoss(_x, _g, _a2, _m);
}

double rangedGoalEquallyValidLoss(const double &_x, const double &_l, const double &_u,
                                  const double &_a1, const double &_a2, const int &_m,
                                  const double &_b, const int &_n)
{
   // xp in [-1, 1]
   double xp = (2*_x - _l - _u)/(_u - _l);

  return ( _a1 + _a2*pow(xp, _m) ) * ( 1.0 - exp( -1.0*pow(xp, _n) / pow(_b, _n) ) ) - 1.0;
}

double rangedGoalPreferredLoss(const double &_x, const double &_l, const double &_u,
                               const double &_g, const double &_c,
                               const double &_a1, const double &_a2, const int &_m,
                               const double &_b, const int &_n)
 {
  return gaussianLoss(_x, _g, _c) +  wallLoss(_x, _l, _u, _a1, _b, _n) +
         polynomialLoss(_x, _g, _a2, _m);

 }                                 
