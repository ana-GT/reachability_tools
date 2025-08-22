
#include <rach_ik/objective.h>

#include <math.h>

struct CostData {
    RobotUnit ru;
    std::string group;
    Eigen::Vector3d goal_pos;
    Eigen::Quaterniond goal_rot;
};

double cost_function(unsigned n, const double *x, double *grad, void *cost_data)
{

    CostData *cd = (CostData *)cost_data;

    // Calculate FK
    Eigen::Vector3d pos; Eigen::Quaterniond rot;
    Eigen::Vector3d pos_diff; Eigen::Quaterniond rot_diff;

    cd->ru.getFK(x, pos, rot);

    pos_diff = (pos - cd->goal_pos).norm();
    rot_diff = ;

    return sqrt( pow(pos_diff, 2) + pow(rot_diff, 2) );

    /*
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }*/

}
