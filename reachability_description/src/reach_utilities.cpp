
#include <reachability_description/reach_utilities.h>

namespace reach_utils {

bool getMinMaxSamples(const reachability_msgs::msg::ReachGraph &_rg,
                      int &_min_samples, int &_max_samples)
{
  if(_rg.points.empty())
    return false;

  _min_samples = 1000;
  _max_samples = 0;

  for(auto pi : _rg.points)
  {
     auto num = pi.samples.size();
     if (num < _min_samples)
       _min_samples = num;
     if (num > _max_samples)
       _max_samples = num;
  }

  return true;
}


double manipValue1(const KDL::JntArray& _q, const std::shared_ptr<KDL::ChainJntToJacSolver> &_jac_solver)
{
  KDL::Jacobian jac(_q.data.size());

  _jac_solver->JntToJac(_q, jac);

  Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
  Eigen::MatrixXd singular_values = svdsolver.singularValues();

  double error = 1.0;
  for (unsigned int i = 0; i < singular_values.rows(); ++i)
    error *= singular_values(i, 0);

  return error;
}

double manipValue2(const KDL::JntArray& _q, const std::shared_ptr<KDL::ChainJntToJacSolver> &_jac_solver)
{
  KDL::Jacobian jac(_q.data.size());

  _jac_solver->JntToJac(_q, jac);

  Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
  Eigen::MatrixXd singular_values = svdsolver.singularValues();

  return singular_values.minCoeff() / singular_values.maxCoeff();
}


/**
 * @function getPlanarTransform
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

/**
 * @function isApproxPlanarTransform
 */
bool isApproxPlanarTransform(const Eigen::Isometry3d &_Tf_start, const Eigen::Isometry3d &_Tf_goal, 
                             double &_tx, double &_ty, double &_yaw, const double &_thresh)
{   
  Eigen::Vector3d zdir_goal, zdir_start;  
  Eigen::Vector3d unit_z(0,0,1);

  double sy, cy;

  zdir_start = _Tf_start.linear().col(2);
  zdir_goal = _Tf_goal.linear().col(2);

  // Projecting
  Eigen::Vector3d pdir_start, pdir_goal;
  
  pdir_start = zdir_start - zdir_start.dot(unit_z) * unit_z;
  pdir_goal = zdir_goal - zdir_goal.dot(unit_z) * unit_z;
  Eigen::Quaterniond q_p; 
  q_p.setFromTwoVectors(pdir_start, pdir_goal);
  Eigen::AngleAxisd aa_p(q_p);
  
  // Yaw angle
  // if angle is w.r.t. -Z, then is negative
  _yaw = unit_z.dot(aa_p.axis()) > 0.9? aa_p.angle() : -1*aa_p.angle();
  
  // Rotate yaw
  Eigen::Matrix3d r_p; r_p = Eigen::AngleAxisd(_yaw, unit_z).toRotationMatrix();
  Eigen::Quaterniond q_est; q_est.setFromTwoVectors( r_p *zdir_start, zdir_goal);
  Eigen::AngleAxisd aa_est(q_est);
  
  if( aa_est.angle() > _thresh )
    return false;
  
       
  auto t_start = _Tf_start.translation();
  auto t_goal = _Tf_goal.translation(); 

  sy = sin(_yaw); cy = cos(_yaw);
  _tx = t_goal(0) - (cy * t_start(0) - sy * t_start(1) );
  _ty = t_goal(1) - (sy * t_start(0) + cy * t_start(1) );          
  return true;
}


} // namespace reach_utils



/**
 * @function stringToTYpe 
 */
TRAC_IK::SolveType stringToType(const std::string &_str)
{
  if(_str == "distance")
    return TRAC_IK::SolveType::Distance;
  else if(_str == "manip1")
    return TRAC_IK::SolveType::Manip1;
  else if(_str == "manip2")
    return TRAC_IK::SolveType::Manip2;
  else if(_str == "speed")
    return TRAC_IK::SolveType::Speed;
  else
    return TRAC_IK::SolveType::Distance;
}

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

sensor_msgs::msg::JointState vectorToJointState(const std::vector<double> &_vec, 
                                                const reachability_msgs::msg::ChainInfo &_ci)
{
  sensor_msgs::msg::JointState js;
  
  if(_ci.joint_names.size() == _vec.size())
  { 
    js.name = _ci.joint_names;
    js.position = _vec;
  }
  
  return js;
}


/**
 * @function jntArrayToMsg 
 */
sensor_msgs::msg::JointState jntArrayToMsg(const KDL::JntArray &_q, 
                                           const reachability_msgs::msg::ChainInfo &_ci)
{
  sensor_msgs::msg::JointState js;
  js.name = _ci.joint_names;
  js.position.resize(_ci.num_joints);
  for(int i = 0; i < js.position.size(); ++i)
    js.position[i] = _q(i);

  return js;
}

/*
double manipPenalty(const KDL::JntArray& q)
{
  double penalty = 1.0;
  for (uint i = 0; i < q.data.size(); i++)
  {
    if (types[i] == KDL::BasicJointType::Continuous)
      continue;
    double range = ub(i) - lb(i);
    penalty *= ((arr(i) - lb(i)) * (ub(i) - arr(i)) / (range * range));
  }
  return std::max(0.0, 1.0 - exp(-1 * penalty));
}*/


