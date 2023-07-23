
#include <reachability_description/reach_utilities.h>


/**
 * @brief Create a KDL frame 
 */
KDL::Frame makeKDLFrame(const double &_x, const double &_y, const double &_z, 
                        const double &_roll, const double &_pitch, const double &_yaw)
{
    KDL::Frame pi;
    pi.p = KDL::Vector(_x, _y, _z);
    pi.M = KDL::Rotation::EulerZYX(_yaw, _pitch, _roll);
    return pi;
}

/**
 * @function jntArrayToVector 
 */
std::vector<double> jntArrayToVector(const KDL::JntArray &_js)
{
  std::vector<double> vec(_js.data.size());
  for(unsigned int i = 0; i < _js.data.size(); ++i)
    vec[i] = _js(i);

  return vec; 
}

/**
 * @function vectorToJntArray
 */
KDL::JntArray vectorToJntArray(const std::vector<double> &_vec)
{
  KDL::JntArray q(_vec.size());
  for(int j = 0; j < _vec.size(); ++j)
    q(j) = _vec[j];

  return q;
}

/**
 * 
 */
Eigen::Isometry3d getPlanarTransform(const double &_x, const double &_y, const double &_yaw)
{
  Eigen::Isometry3d Tf;
  Tf.setIdentity();
  Tf.translation() = Eigen::Vector3d(_x, _y, 0);
  
  Eigen::Matrix3d rot; 
  rot = Eigen::AngleAxisd(_yaw, Eigen::Vector3d::UnitZ());
  Tf.linear() = rot;

  return Tf;
}