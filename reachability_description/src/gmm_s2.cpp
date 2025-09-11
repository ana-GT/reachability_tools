

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
    Eigen::Vector2d u_tm;
    
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
  Eigen::Vector2d mean(const std::vector<Eigen::Vector3d> &_xs, 
  		       const Eigen::Vector3d &_u_m)
  {    
    Eigen::Vector2d u = Eigen::Vector2d::Zero();
    
    for(int i = 0; i < _xs.size(); ++i)
     u += Log(_u_m, _xs[i]);
    
    u /= (double)(_xs.size());
     
    return u;
  }
  
  /**
   * @function d : Metric product. Not sure of this one 
   */  
  double d(const Eigen::Vector3d &_x, const Eigen::Vector3d &_y)
  {
     return acos(_x.transpose()*_y);
  }

  /**
   * @function u = Log_x(y), x in M, y in M, u in TM 
   */
  Eigen::Vector2d Log(const Eigen::Vector3d &_x, const Eigen::Vector3d &_y)
  {
     Eigen::Vector2d u;
     Eigen::Vector3d y;
     
     Eigen::Matrix3d Rp;
     Rp = Eigen::Quaterniond().setFromTwoVectors(_x, Eigen::Vector3d(0,0,1));

     y = Rp * _y;
     y.normalize();
     double theta = acos(y.z());
     
     double t_sint;
     if( fabs(theta) < 1e-5)
       t_sint = 1.0;
     else 
       t_sint = theta/sin(theta);
       
     u = Eigen::Vector2d(y.x()*t_sint, y.y() * t_sint);
     return u;
  }

  
  /** 
   * @brief y = Exp_x(u) x in M, u in TM, return value y in M 
   */
  Eigen::Vector3d Exp(Eigen::Vector3d &_x, Eigen::Vector2d &_u)
  {
  
   // Calculate Rp: Rotation between x and north pole
   Eigen::Matrix3d Rp;
   Rp = Eigen::Quaterniond().setFromTwoVectors(_x, Eigen::Vector3d(0,0,1));
   
   // Get exp
   Eigen::Vector3d y;   
   double u_norm = _u.norm();
   y = Eigen::Vector3d( _u.x() * sin(u_norm)/u_norm, _u.y() * sin(u_norm)/u_norm, cos(u_norm) ); 
   y = Rp.inverse() * y;
   // Normalize for good measure   
   y.normalize();

   return y;
  }

  ////////////////////////////////////////////////
  // GMM
  ////////////////////////////////////////////////
  GMM::GMM()
  {
    num_iterations_ = 100;
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
    if(!initializeParameters(_k))
      return false;

    for(int i = 0; i < num_iterations_; ++i)
    {
      std::vector<Gaussian> param_old;
      param_old = params_;
      
      if(!Estep())
      { 
        RCLCPP_INFO(rclcpp::get_logger("gmm"), "[%d] Error in Estep calculation", i); 
        return false;
      }  
      Mstep();

      double err_avg = 0;
      for(int k = 0; k < k_; ++k) {
        
        Eigen::Vector3d diff; 
        diff = param_old[k].u - params_[k].u;
        err_avg += diff.norm();
        }
        err_avg /= (double) k_;
	if(err_avg < 0.01)
	{
	  RCLCPP_INFO(rclcpp::get_logger("gmm"), "Converged in iter %d" , i);
	  break;
	}
      } // for i
    
    _ps = this->xs_;
    _gs = this->params_;

    return true;
  }

  /**
   * @funcion Estep
   */
  bool GMM::Estep()
  {
    int N = xs_.size();

    for(auto &x : xs_)
    {
      double sum_nums = 0;
      for(int j = 0; j < k_; ++j)
      { 
        double d = normalDist(x, params_[j]);
        if( std::isnan(d) )
          return false;
          
        x.gk[j] = params_[j].pi_k * d;

        sum_nums += x.gk[j];
      }

      for(int j = 0; j < k_; ++j)
        x.gk[j] /= sum_nums;
    }
    
    return true;
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
      Eigen::Vector2d ut_new;
      Eigen::Vector3d u_new, u_last;
      u_last = params_[k].u;
      ut_new = Eigen::Vector2d::Zero();

      for(auto x : xs_)
         ut_new += x.gk[k] * Log(u_last, x.x);

      ut_new /= Nk[k]; //(Nk[k]/(double)N);
      u_new = Exp(u_last, ut_new);

      // Normalize for good measure
      u_new.normalize();

      // Store
      params_[k].u = u_new;
    }

    //-- Calculate new Sk
    for(int k = 0; k < k_; ++k)
    {
      Eigen::Matrix2d S_new;
      S_new = Eigen::Matrix2d::Zero();

      for(auto x : xs_)
      {
        Eigen::Vector2d log;
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
    Eigen::Vector2d log;

    log = Log(_params.u, _x.x);

    return exp( -0.5*log.transpose() *_params.S.inverse()*log ) / sqrt( pow(2*M_PI, d)*_params.S.determinant() );
  }

  bool GMM::initializeParameters(const int &_k)
  {
    k_ = _k;
    
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
      params_[i].S = Eigen::Matrix2d::Identity();
      params_[i].pi_k = 1.0/(double)_k;
    }
    
    for(auto &x : xs_)
       x.gk.resize(k_);

    return true;
  }


  KMedoids::KMedoids()
  {
    num_iterations_ = 100;
    medoids_thresh_ = 0.001;
  }

  /**
   * @function
   */
  void KMedoids::addPoints(const std::vector<Eigen::Vector3d> &_x)
  {
    x_.clear();
    for(auto xi : _x)
    {
      KMedoidPoint p;
      p.x = xi;
      x_.push_back(p);
    }
  }
  
  bool KMedoids::kmedoids(const unsigned int &_k, 
            std::vector<Eigen::Vector3d> &_u,
            std::vector<unsigned int> &_indices)
  {
    bool converged = false;

    if(!initializeParameters(_k))
      return false;

    for(int i = 0; i < num_iterations_; ++i)
    {
      // Calculate assignments
      calculateAssignments();
      // Recalculate means
      if(calculateMedoids())
      {
        RCLCPP_INFO(rclcpp::get_logger("med"), "Converged at iteration %d", i);
        converged = true;
        break;
      }      
    }

    // Return assignments and means
    if(converged)
    {
      _u.clear();
      _indices.clear();

      _u = u_;
      for(auto xi : x_)
        _indices.push_back(xi.k);
    }

    return converged;
  }

  void KMedoids::calculateAssignments()
  {
    int min_index;
    double min_val, dist;

    for(auto &xi : x_)
    {
      min_val = 10000;
      min_index = -1;

      for(int k = 0; k < k_; ++k)
      {
        dist = d(u_[k], xi.x);
        if(dist < min_val)
        {
          min_val = dist;
          min_index = k;
        }
      }

      xi.k = min_index;
    }


  }

  /**
   * @return true if converged, false otherwise
   */
  bool KMedoids::calculateMedoids()
  {
    std::vector<Eigen::Vector3d> u_old;
    u_old = u_;

    for(int k = 0; k < k_; ++k)
    {
      Eigen::Vector2d num(0,0);
      int den = 0;
      int index = 0;
    	for(auto xi : x_)
    	{    	   
    	   if(xi.k == k)
    	   {
    	      num += Log(u_[k], xi.x);
    	      den += 1;
    	   }
    	}

      num /= (double) den;    	
    	u_[k] = Exp(u_[k], num);
    } // for k


    // Check convergence
    double u_diff, sum_diff;
    sum_diff = 0;
    for(int k = 0; k < k_; ++k)
    {
      u_diff = (u_[k] - u_old[k]).norm();
      sum_diff += u_diff;
    }
    
    return (sum_diff / (double)k_) < medoids_thresh_;

  }

  bool KMedoids::initializeParameters(const unsigned int &_k)
  {

    k_ = _k;

    u_.clear();
    u_.resize(_k);

    if(k_ == 0)
      return false;

    if(x_.size() < _k)
      return false;

    int index = 0;
    for(int i = 0; i < _k; ++i)
    {
      index = (int)std::floor((double)(i)* (double)(x_.size() - 1)/(double)_k );
      u_[i] = x_[index].x;
    }
    
    return true;
  }

} // namespace s2
