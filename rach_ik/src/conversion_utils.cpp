
#include <rach_ik/conversion_utils.h>

void jointStateToVector(const sensor_msgs::msg::JointState &_js,
                        std::vector<double> &_x)
{
    _x.clear();
    for(auto ji : _js.position)
        _x.push_back(ji);
}

/**
 * @function getTfPlanar
 */
Eigen::Isometry3d getTfPlanar(const double &_x, const double &_y, const double &_alpha)
{
 Eigen::Isometry3d Tfx;
 Tfx.setIdentity();
 
 Tfx.translation() = Eigen::Vector3d(_x, _y, 0);
 Eigen::Matrix3d m;
 m = Eigen::AngleAxisd(_alpha, Eigen::Vector3d(0, 0, 1));
 Tfx.linear() = m;
 
 return Tfx;
}

/**
 * @function fromTfPlanar
 */
bool fromTfPlanar(const Eigen::Isometry3d &_Tfx, double &_x, double &_y, double &_alpha) {

 _x = _Tfx.translation().x();
 _y = _Tfx.translation().y();
 
 Eigen::AngleAxisd aa(_Tfx.linear());
 
 _alpha = aa.angle();
 return true;
}
