
#include <rach_ik_plugins/constraints/constraint_elbow_wrist.h>
#include <cfloat>

 double elbow_wrist_cosine(const std::vector<double> &_x, void *_data)
 {
    ConstraintData *cd = (ConstraintData *) _data;

    // Constraint: Angle between Up vector (e.g. Z) with vector from elbow  to wrist must be < 90. Cos(angle) must be positive 
    // - cos(angle) < 0

    // 1. Find poses of elbow and wrist
    KDL::JntArray q; KDL::Frame tf_elbow, tf_wrist;

    q.resize(_x.size());
    for(int i = 0; i < _x.size(); i++)
        q(i) = _x[i];

    cd->fk_solver->JntToCart(q, tf_elbow, cd->elbow_index);
    cd->fk_solver->JntToCart(q, tf_wrist, cd->wrist_index);

    // Get vector 
    KDL::Vector elbow_wrist =  tf_wrist.p - tf_elbow.p;
    elbow_wrist.Normalize();

    // Get cosine between this and KDL::Vector3(0, 0, 1)
    double cos_angle = KDL::dot(elbow_wrist, KDL::Vector(0,0,1));

    // Return 
    return cos_angle;
 }   

double constraint_elbow_down(const std::vector<double> &_x, std::vector<double> &_grad, void *_data)
{
    double result;
    result = -1 * elbow_wrist_cosine(_x, _data);

    // Grad
    std::vector<double> vals(_x);
    double jump = FLT_EPSILON;

    if (!_grad.empty()) {

      double v1;
      for (uint i = 0; i < _x.size(); i++)
      {
        double original = vals[i];

        vals[i] = original + jump;

        v1 = -1 * elbow_wrist_cosine(_x, _data);

        vals[i] = original;
        _grad[i] = (v1 - result) / (2 * jump);
      }

    }

    return result;
 }
