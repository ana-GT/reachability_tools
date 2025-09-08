/**
 * @file reach_utilities.h 
 */
#pragma once

#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>
#include <reachability_msgs/msg/chain_info.hpp>
#include <reachability_msgs/msg/reach_graph.hpp>

#include <trac_ik/trac_ik.hpp>

KDL::Frame makeKDLFrame(const double &_x, const double &_y, const double &_z, 
                        const double &_roll, const double &_pitch, const double &_yaw);

std::vector<double> jntArrayToVector(const KDL::JntArray &_js);
KDL::JntArray vectorToJntArray(const std::vector<double> &_vec);
sensor_msgs::msg::JointState vectorToJointState(const std::vector<double> &_vec, 
                                                const reachability_msgs::msg::ChainInfo &_ci);
Eigen::Isometry3d getPlanarTransform(const double &_x, const double &_y, const double &_yaw);

sensor_msgs::msg::JointState jntArrayToMsg(const KDL::JntArray &_q, 
                                           const reachability_msgs::msg::ChainInfo &_ci);

TRAC_IK::SolveType stringToType(const std::string &_str);                                           

/////////////////////////////

namespace reach_utils {

bool getMinMaxSamples(const reachability_msgs::msg::ReachGraph &_rg,
                      int &_min_samples, int &_max_samples);


double manipValue1(const KDL::JntArray& _q, const std::shared_ptr<KDL::ChainJntToJacSolver> &_jac_solver);
double manipValue2(const KDL::JntArray& _q, const std::shared_ptr<KDL::ChainJntToJacSolver> &_jac_solver);

}
