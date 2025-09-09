

#include <reachability_description/gmm_s2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

namespace s2 {

  bool minimizeCentroid(const std::vector<Eigen::Vector3d> &_xs, 
                         Eigen::Vector3d &_u, 
                         const int &_num_iters, 
                         const double &_threshold)
  {
    bool converged = false;
    
    Eigen::Vector3d u_m, u_m_new;
    Eigen::Vector3d u_tm;
    
    // Let's initialize u_m randomly as the first 
    u_m = _xs[8];
       
    for(int i = 0; i < _num_iters; ++i)
    {
       u_tm = mean(_xs, u_m);
  
       u_m_new = Exp(u_m, u_tm);
       
       if( (u_m_new - u_m).norm() < _threshold)
       {RCLCPP_INFO(rclcpp::get_logger("GMMS2"), "Converged %d/%d!!!!!", i, _num_iters);
         converged = true;
         _u = u_m;
         break;
       } 
       
       u_m = u_m_new;
    }
    
    return converged;
  }

  /**
   * @function mean : Mean 
   */
  Eigen::Vector3d mean(const std::vector<Eigen::Vector3d> &_xs, 
  		       const Eigen::Vector3d &_u_m)
  {    
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    
    for(int i = 0; i < _xs.size(); ++i)
     u += Log(_u_m, _xs[i]);
    
    u /= (double)(_xs.size());
     
    return u;
  }
  
  /**
   * @function d : Metric product 
   */  
  double d(const Eigen::Vector3d &_x, const Eigen::Vector3d &_y)
  {
     return acos(_x.transpose()*_y);
  }

  /**
   * @function u = Log_x(y), x in M, y in M, u in TM 
   */
  Eigen::Vector3d Log(const Eigen::Vector3d &_x, const Eigen::Vector3d &_y)
  {
     Eigen::Vector3d u, v;
     
     if( (_x - _y).norm() < 0.000001)
       return _x;
     
     v = (_y - _x.transpose()*_y*_x);
     u = d(_x, _y)* v / v.norm();
     
     return u;
  }

  
  /** 
   * @brief y = Exp_x(u) x in M, u in TM, return value y in M 
   */
  Eigen::Vector3d Exp(Eigen::Vector3d &_x, Eigen::Vector3d &_u)
  {
   Eigen::Vector3d y;
   
   double u_norm = _u.norm();
   y = _x*cos(u_norm) + (_u/u_norm)*sin(u_norm);
   
   // Normalize for good measure   
   y.normalize();

   return y;
  }

  ////////////////////////////////////////////////
  // GMM
  ////////////////////////////////////////////////
  GMM::GMM()
  {
    num_iterations_ = 20;
  }
  
  void GMM::addPoints(const std::vector<Eigen::Vector3d> &_x)
  {
    xs_.clear();
    xs_.resize(_x.size());

    int i = 0;
    for(auto xi : _x)
    {
      xs_[i].x = xi; 
      i++;
    }
  }

  bool GMM::EM(const int &_k, std::vector<Gaussian> &_gs, std::vector<GmmPoint> &_ps)
  {
    k_ = _k;

    if(!initializeParameters(_k))
      return false;

        RCLCPP_INFO(rclcpp::get_logger("gmm"), "Start EM");
    for(int i = 0; i < num_iterations_; ++i)
    {
      Estep();
      Mstep();
      for(int k = 0; k < k_; ++k) {
        Eigen::Vector3d u;
        u = params_[k].u;
        RCLCPP_INFO(rclcpp::get_logger("gmm"), "Iter[%d] U: %f %f %f pk: %f",
        u.x(), u.y(), u.z(), params_[k].pi_k);
      }
    }

    _ps = this->xs_;
    _gs = this->params_;

    return true;
  }

  /**
   * @funcion Estep
   */
  void GMM::Estep()
  {
    int N = xs_.size();

    for(auto &x : xs_)
    {
      double sum_nums = 0;
      for(int j = 0; j < k_; ++j)
      { 
        x.gk[j] = params_[j].pi_k * normalDist(x, params_[j]);
        sum_nums += x.gk[j];
      }

      for(int j = 0; j < k_; ++j)
        x.gk[j] /= sum_nums;
    }
  }

/**
 * @brief Mstep
 */
  void GMM::Mstep() {

    int N = xs_.size();

    //-- Calculate Nk
    double Nk[k_];
    for(int k = 0; k < k_; ++k)
    {
      Nk[k] = 0.0;
      for(auto x : xs_)
        Nk[k] += x.gk[k];
    }
    
    //-- Calculate new uk
    for(int k = 0; k < k_; ++k)
    {
      Eigen::Vector3d u_new, ut_new, u_last;
      u_last = params_[k].u;
      ut_new = Eigen::Vector3d::Zero();

      for(auto x : xs_)
         ut_new += x.gk[k] * Log(u_last, x.x);

      ut_new /= Nk[k];
      u_new = Exp(u_last, ut_new);

      // Normalize for good measure
      u_new.normalize();

      // Store
      params_[k].u = u_new;
    }

    //-- Calculate new Sk
    for(int k = 0; k < k_; ++k)
    {
      Eigen::Matrix3d S_new;
      S_new = Eigen::Matrix3d::Zero();

      for(auto x : xs_)
      {
        Eigen::Vector3d log;
        log = Log(params_[k].u, x.x);
        
        S_new += x.gk[k]*log*log.transpose();
      }

      S_new /= Nk[k];

      // Store
      params_[k].S = S_new;
    }

    //-- Calculate pi_k
    for(int k = 0; k < k_; ++k)
      params_[k].pi_k = Nk[k] / (double) N;


  }

  double GMM::normalDist(const GmmPoint &_x, Gaussian _params)
  {
    int d = 2;
    Eigen::Matrix3d S;
    Eigen::Vector3d log;

    log = Log(_params.u, _x.x);

    return exp( -0.5*log.transpose() *_params.S.inverse()*log ) / sqrt( pow(2*M_PI, d)*S.determinant() );
  }

  bool GMM::initializeParameters(const int &_k)
  {
    params_.clear();
    params_.resize(_k);

    if(_k == 0)
      return false;

    if(xs_.size() < _k)
      return false;

    int index = 0;
    for(int i = 0; i < _k; ++i)
    {
      index = (double)(i)* (double)(xs_.size() - 1)/(double)_k ;
      params_[i].u = xs_[index].x;
      params_[i].S = Eigen::Matrix3d::Identity();
      params_[i].pi_k = 1.0/(double)_k;
    }

    return true;
  }


} // namespace s2
