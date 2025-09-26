
#pragma once

#include <Eigen/Geometry>
#include <vector>

namespace se2 {

  Eigen::VectorXd Log(const Eigen::Isometry3d &_x, const Eigen::Isometry3d &_y);
  Eigen::Isometry3d Exp(Eigen::Isometry3d &_x, Eigen::VectorXd &_u);
  double d(const Eigen::Isometry3d &_x, const Eigen::Isometry3d &_y);

//  Eigen::Vector2d mean(const std::vector<Eigen::Vector3d> &_xs, 
//  		       const Eigen::Vector3d &_u_m);
  		       
//  bool minimizeCentroid(const std::vector<Eigen::Vector3d> &_xs, 
//                         Eigen::Vector3d &_u, 
//                         const int &_num_iters = 20, 
//                         const double &_threshold = 0.0001);

  /**
   * @struct Gaussian 
   */
//  struct Gaussian {
//    Eigen::Vector3d u;
//    Eigen::Matrix2d S;
//    double pi_k;
    
//    Gaussian() {
//      u = Eigen::Vector3d::Zero();
//      S = Eigen::Matrix2d::Identity();
//      pi_k = 1.0;
//    }
//  };

//  struct GmmPoint {
//    Eigen::Vector3d x;
//    std::vector<double> gk; // gamma K  
//  };

  /**
   * @class GMM 
   */
 /* class GMM {

    public:

      GMM();
      void addPoints(const std::vector<Eigen::Vector3d> &_x);
      bool initializeParameters(const int &_k);
      bool EM(const int &_k, std::vector<Gaussian> &_gs, std::vector<GmmPoint> &_ps);
      bool Estep();
      void Mstep();
      double normalDist(const GmmPoint &_x, Gaussian _params);      

    protected:
      int num_iterations_;

      std::vector<Gaussian> params_;
      std::vector<GmmPoint> xs_;
      int k_;
  };*/

  struct KMedoidPoint{
    Eigen::Isometry3d x;
    unsigned int k;
  };

  enum ConvergeState {
    CONVERGED,
    NON_CONVERGED,
    IS_NAN
  };

  /**
   * @class kmedoids
   */
  class KMedoids {

    public:
      KMedoids();
      void addPoints(const std::vector<Eigen::Isometry3d> &_x);
      bool kmedoids(const unsigned int &_k,
            std::vector<Eigen::Isometry3d> &_u,
            std::vector<unsigned int> &_indices);

    protected:

      bool initializeParameters(const unsigned int &_k);
      void calculateAssignments();
      ConvergeState calculateMedoids();

      int num_iterations_;
      double medoids_thresh_;
      
      std::vector<Eigen::Isometry3d> u_;
      std::vector<KMedoidPoint> x_;
      int k_;
  };

} // namespace se2


