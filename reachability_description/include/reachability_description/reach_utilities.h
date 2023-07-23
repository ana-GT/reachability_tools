/**
 * @file reach_utilities.h 
 */
#pragma once

#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <kdl/jntarray.hpp>
#include <vector>

KDL::Frame makeKDLFrame(const double &_x, const double &_y, const double &_z, 
                        const double &_roll, const double &_pitch, const double &_yaw);

std::vector<double> jntArrayToVector(const KDL::JntArray &_js);
KDL::JntArray vectorToJntArray(const std::vector<double> &_vec);

Eigen::Isometry3d getPlanarTransform(const double &_x, const double &_y, const double &_yaw);