
#pragma once

#include <Eigen/Geometry>
#include <vector>

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

  /**
   * @class Gaussian 
   */
  struct Gaussian {
    Eigen::Vector3d u;
    Eigen::Matrix3d S;
    double pi_k;
  };

  struct GmmPoint {
    Eigen::Vector3d x;
    std::vector<double> gk; // gamma K  
  };

  /**
   * @class GMM 
   */
  class GMM {

    public:

      GMM();
      void addPoints(const std::vector<Eigen::Vector3d> &_x);
      bool initializeParameters(const int &_k);
      bool EM(const int &_k, std::vector<Gaussian> &_gs, std::vector<GmmPoint> &_ps);
      void Estep();
      void Mstep();
      double normalDist(const GmmPoint &_x, Gaussian _params);      

    protected:
      int num_iterations_;

      std::vector<Gaussian> params_;
      std::vector<GmmPoint> xs_;
      int k_;
  };

} // namespace s2


