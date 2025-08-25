
#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Geometry>
#include <vector>

void jointStateToVector(const sensor_msgs::msg::JointState &_js,
                        std::vector<double> &_x);

Eigen::Isometry3d getTfPlanar(const double &_x, const double &_y, const double &_alpha);
bool fromTfPlanar(const Eigen::Isometry3d &_Tfx, double &_x, double &_y, double &_alpha);
