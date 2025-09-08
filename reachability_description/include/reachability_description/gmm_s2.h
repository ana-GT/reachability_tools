
#pragma once

#include <Eigen/Geometry>

namespace s2 {

  Eigen::Vector3d Log(const Eigen::Vector3d &_x, const Eigen::Vector3d &_y);
  Eigen::Vector3d Exp(Eigen::Vector3d &_x, Eigen::Vector3d &_u);
  double d(const Eigen::Vector3d &_x, const Eigen::Vector3d &_y);

  Eigen::Vector3d mean(const std::vector<Eigen::Vector3d> &_xs, 
  		       const Eigen::Vector3d &_u_m);
  		       
  bool minimizeCentroid(const std::vector<Eigen::Vector3d> &_xs, 
                         Eigen::Vector3d &_u, 
                         const int &_num_iters = 20, 
                         const double &_threshold = 0.0001); 		       
}
