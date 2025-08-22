
#include <rach_ik/objective.h>

#include <math.h>


double cost_function(unsigned n, const double *x, double *grad, void *cost_data)
{

    CostData *cd = (CostData *)cost_data;

    // Calculate FK
    Eigen::Vector3d pos; Eigen::Quaterniond rot;
    Eigen::Vector3d pos_diff; Eigen::Quaterniond rot_diff;


    std::vector<double> q;
    Eigen::Vector3d pos; Eigen::Quaterniond rot;
    q.resize(n);
    for(int i = 0; i < n; ++i)
        q[i] = x[i];

//    cd->re.getFK(cd->goal.group, q, pos, rot);

    pos_diff = (pos - cd->goal.pos).norm();
    rot_diff = cd->goal.rot.eigen2_dot(rot);

    return sqrt( pow(pos_diff, 2) + pow(rot_diff, 2) );

    /*
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }*/

}
