
#include <rach_ik_plugins/objectives/objective_ee_diff.h>
#include <rclcpp/rclcpp.hpp>
#include <cfloat>

double cost_ee_diff_function(const std::vector<double> &x, std::vector<double> &grad, void *objective_data)
{
  double dlin, drot;
  calculateEEDiff(x, dlin, drot, objective_data);

  double error = (dlin*dlin + drot*drot);
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
      v1 = (dlin*dlin + drot*drot);

      vals[i] = original;
      grad[i] = (v1 - error) / (2 * jump);
    }
  }

  return error;
}


