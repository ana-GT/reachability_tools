
#include <rach_ik_plugins/objectives/objective_mobile_ee_diff.h>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cfloat>


void calculateMobileEEDiff(const std::vector<double> &x, 
                           double &_dlin, double &_drot, 
                           void *objective_data ) {

    int n = x.size() - 3;
    MobileObjectiveData *od = (MobileObjectiveData *)objective_data;

    // Calculate FK
    Eigen::Vector3d pos; Eigen::Quaterniond rot;
  
    KDL::JntArray q; KDL::Frame Tfx_root_ee;
    Eigen::Isometry3d Tf_root_ee, Tf_ref_ee;
    double qx, qy, qz, qw;

    q.resize(n);
    for(int i = 0; i < n; i++)
        q(i) = x[i];

    int res = od->fk_solver->JntToCart(q, Tfx_root_ee);
    if(res < 0)
      RCLCPP_INFO(rclcpp::get_logger("iko"), "Something went horribly wrong when calculating FK: %d", res);
      
    tf2::transformKDLToEigen(Tfx_root_ee, Tf_root_ee);

    // Full FK
    Tf_ref_ee = getTfPlanar(x[n], x[n+1], x[n+2]) * od->Tf_base_root * Tf_root_ee;
 
    pos = Tf_ref_ee.translation();
    rot = Tf_ref_ee.linear();
    
    _dlin = (pos - od->goal_pos).norm();
    _drot = Eigen::AngleAxisd(rot*od->goal_rot.inverse()).angle();

}

double cost_mobile_ee_diff_function(const std::vector<double> &x, std::vector<double> &grad, void *objective_data)
{
  double dlin, drot;
  calculateMobileEEDiff(x, dlin, drot, objective_data);

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

      calculateMobileEEDiff(vals, dlin, drot, objective_data);
      v1 = (dlin*dlin + drot*drot);

      vals[i] = original;
      grad[i] = (v1 - error) / (2 * jump);
    }
  }

  return error;
}


