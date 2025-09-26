
#include <rach_ik_plugins/constraints/constraint_wrist_front.h>
#include <cfloat>
#include <rclcpp/rclcpp.hpp>
namespace constraint_wrist {

 double wrist_front(const std::vector<double> &_x, void *_data)
 {
    int n = _x.size() - 3;
    ConstraintData *cd = (ConstraintData *) _data;

    // 1. Find poses of wrist
    KDL::JntArray q; KDL::Frame tf_wrist;

    q.resize(n);
    for(int i = 0; i < n; i++)
        q(i) = _x[i];

    int res = cd->fk_solver->JntToCart(q, tf_wrist, cd->wrist_index);
    if(res < 0)
      RCLCPP_INFO(rclcpp::get_logger("iko"), "Something went horribly wrong when calculating FK");

    // Get vector 
    //KDL::Vector elbow_wrist =  tf_wrist.p - tf_elbow.p;
    //elbow_wrist.Normalize();

    // Get cosine between this and KDL::Vector3(0, 0, 1)
    //double cos_angle = KDL::dot(elbow_wrist, KDL::Vector(0,0,1));

    // Return 
    return  0.3 - fabs(tf_wrist.p.y());
 }   

double constraint_wrist_front(const std::vector<double> &_x, std::vector<double> &_grad, void *_data)
{
    double result;
    result = -1 * wrist_front(_x, _data);

    // Grad
    std::vector<double> vals(_x);
    double jump = FLT_EPSILON;

    if (!_grad.empty()) {

      double v1;
      for (uint i = 0; i < _x.size(); i++)
      {
        double original = vals[i];

        vals[i] = original + jump;

        v1 = -1 * wrist_front(_x, _data);

        vals[i] = original;
        _grad[i] = (v1 - result) / (2 * jump);
      }

    }

    return result;
 }
 
} // namespace constraint_wrist
