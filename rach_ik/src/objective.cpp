
#include <rach_ik/objective.h>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <cfloat>

void calculateEEDiff(const std::vector<double> &x, 
                     double &_dlin, double &_drot, 
                     void *objective_data ) {

    int n = x.size();
    ObjectiveData *od = (ObjectiveData *)objective_data;

    // Calculate FK
    Eigen::Vector3d pos; Eigen::Quaterniond rot;
  
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
      RCLCPP_INFO(rclcpp::get_logger("iko"), "curr: %f, %f, %f -- goal: %f, %f, %f", tfx.p.x(), tfx.p.y(),tfx.p.z(), od->goal_pos.x(), od->goal_pos.y(), od->goal_pos.z() );
    _dlin = (pos - od->goal_pos).norm();
    _drot = Eigen::AngleAxisd(rot*od->goal_rot.inverse()).angle();

}
