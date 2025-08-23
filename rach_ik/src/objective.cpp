
#include <rach_ik/objective.h>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <cfloat>

void calculateError(const std::vector<double> &x, double &_error, void *objective_data ) {

    int n = x.size();
    ObjectiveData *od = (ObjectiveData *)objective_data;

    // Calculate FK
    Eigen::Vector3d pos; Eigen::Quaterniond rot;
    double pos_diff; double rot_diff;
 
    KDL::JntArray q; KDL::Frame tfx;
    double qx, qy, qz, qw;

    q.resize(n);
    for(int i = 0; i < n; i++)
        q(i) = x[i];

    int res = od->fk_solver->JntToCart(q, tfx);
    if(res < 0)
      RCLCPP_INFO(rclcpp::get_logger("iko"), "Something went horribly wrong when calculating FK: %d", res);

 
    pos = Eigen::Vector3d(tfx.p.x(), tfx.p.y(),tfx.p.z());
    tfx.M.GetQuaternion(qx, qy, qz, qw);
    rot = Eigen::Quaterniond(qw, qx, qy, qz);

    pos_diff = (pos - od->goal_pos).norm();
    rot_diff = 0; //od->goal_rot.eigen2_dot(rot);


    _error = sqrt( pow(pos_diff, 2) + pow(rot_diff, 2) );
}

double cost_function(const std::vector<double> &x, std::vector<double> &grad, void *objective_data)
{
    double error;
    calculateError(x, error, objective_data);


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
      calculateError(vals, v1, objective_data);

      vals[i] = original;
      grad[i] = (v1 - error) / (2 * jump);
    }
  }

  return error;
}
