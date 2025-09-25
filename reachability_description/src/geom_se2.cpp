

#include <reachability_description/geom_se2.h>
#include <rclcpp/rclcpp.hpp>

namespace se2 {
/*
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
  }*/

  /**
   * @function mean : Mean 
   */
 /* Eigen::Vector2d mean(const std::vector<Eigen::Vector3d> &_xs, 
  		       const Eigen::Vector3d &_u_m)
  {    
    Eigen::Vector2d u = Eigen::Vector2d::Zero();
    
    for(auto xi : _xs)
     u += Log(_u_m, xi);
    
    u /= (double)(_xs.size());
     
    return u;
  }*/
  
  /**
   * @function d : Metric product. Not sure of this one
   * @brief For now, just use geometric distance 
   */  
  double d(const Eigen::Isometry3d &_x, const Eigen::Isometry3d &_y)
  {
     return (_x.translation() - _y.translation()).norm();
  }

  /**
   * @function u = Log_x(y), x in M, y in M, u in TM 
   */
  Eigen::Vector3d Log(const Eigen::Isometry3d &_x, const Eigen::Isometry3d &_y)
  {
  
     Eigen::Vector3d u;
     Eigen::AngleAxisd aa(_y.linear());
     double yaw;
     yaw = aa.axis().dot(Eigen::Vector3d::UnitZ()) > 0.9? aa.angle() : -1 * aa.angle();
       
     u << _y.translation()(0), _y.translation()(1), yaw; 
     
     return u;
  }

  
  /** 
   * @brief y = Exp_x(u) x in M, u in TM, return value y in M 
   */
  Eigen::Isometry3d Exp(Eigen::Isometry3d &_x, Eigen::Vector3d &_u)
  {
  
   Eigen::Isometry3d y;
   y.setIdentity();
   y.translation() << _u(0), _u(1), 0;
   y.linear() = Eigen::AngleAxisd(_u(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  
   return y;
  }

  ////////////////////////////////////////////////
  // GMM
  ////////////////////////////////////////////////
/*  GMM::GMM()
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
  }*/

  /**
   * @funcion Estep
   */
 /* bool GMM::Estep()
  {
    int N = xs_.size();

    for(auto &x : xs_)
    {
      double sum_nums = 0;
      for(int j = 0; j < k_; ++j)
      { 
        double d = normalDist(x, params_[j]);
        if( std::isnan(d) )
        {
          RCLCPP_ERROR(rclcpp::get_logger("gmm"), "Error in Estep for point (%f %f %f) with norm: %f with u: (%f %f %f) and norm: %f", 
            x.x.x(), x.x.y(), x.x.z(), x.x.norm(), params_[j].u.x(), params_[j].u.y(), params_[j].u.z(), params_[j].u.norm());
          return false;
        }  
        x.gk[j] = params_[j].pi_k * d;

        sum_nums += x.gk[j];
      }

      for(int j = 0; j < k_; ++j)
        x.gk[j] /= sum_nums;
    }
    
    return true;
  }*/

/**
 * @brief Mstep
 */
/*  void GMM::Mstep() {

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
        
        Eigen::Matrix2d par;
        par = x.gk[k]*log*log.transpose();
        S_new += par;
      }
      S_new /= Nk[k];

      // Store
      params_[k].S = S_new;
    }

    //-- Calculate pi_k
    for(int k = 0; k < k_; ++k)
      params_[k].pi_k = Nk[k] / (double) N;

  }*/

  /**
   * @function normalDist
   */
 /* double GMM::normalDist(const GmmPoint &_x, Gaussian _params)
  {
    int d = 2;
    Eigen::Vector2d log;

    log = Log(_params.u, _x.x);
    
    double nd = exp( -0.5*log.transpose() *_params.S.inverse()*log ) / sqrt( pow(2*M_PI, d)*_params.S.determinant() );
    if(std::isnan(nd))
    {
       RCLCPP_INFO_STREAM(rclcpp::get_logger("gmm"), "Log: (" << log.transpose() << ") and S: \n"<< _params.S.matrix() << "\n and det: " << _params.S.determinant());    
    } 
    return nd;
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
      index = std::min( (int)std::ceil ( (double)(i)* (double)(xs_.size() - 1)/(double)_k ), (int)(xs_.size() - 1) );
      params_[i].u = xs_[index].x;
      params_[i].S = Eigen::Matrix2d::Identity();
      params_[i].pi_k = 1.0/(double)_k;
    }
    
    for(auto &x : xs_)
       x.gk.resize(k_);

    return true;
  }*/

  /**
   * @function KMedoids
   * @brief Constructor
   */
  KMedoids::KMedoids()
  {
    num_iterations_ = 100;
    medoids_thresh_ = 0.001;
  }

  /**
   * @function
   */
  void KMedoids::addPoints(const std::vector<Eigen::Isometry3d> &_x)
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
            std::vector<Eigen::Isometry3d> &_u,
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
      ConvergeState cs = calculateMedoids(); 
      if( cs == CONVERGED)
      {
        converged = true;
        break;
      } else if(cs == IS_NAN)
      {
      	converged = false;
      	break;
      }    
    }

    // Return assignments and means
    if(converged)
    {
      _u.clear();
      _indices.clear();

      // Check whether there are any unassigned kmedoids (likely to happen with small clusters)
      std::vector<unsigned int> assign_size(k_, 0);
      std::vector<int> assign_labels(k_, -1);
      for(unsigned int k = 0; k < k_; ++k)
      {
      	for(auto x : x_)
      	{
      	   if(x.k == k)
      	     assign_size[k]++;
      	}
      }

      int label = 0;
      for(int i = 0; i < assign_size.size(); ++i)
      {
      	if(assign_size[i] > 0)
      	{
      	  assign_labels[i] = label;
      	  label++;
      	}
      }

      //if(label != k_)
      //   RCLCPP_WARN(rclcpp::get_logger("DEBUG!"), "K: %d label < max: %d", k_, label);
    
      // Store us
      for(int i = 0; i < k_; ++i)
      {
         if(assign_size[i] > 0)
           _u.push_back(u_[i]);
      }

      
      for(auto xi : x_)
      {
        if(assign_labels[xi.k] == -1)
          RCLCPP_ERROR(rclcpp::get_logger("HORROR"), "Something bad happened with assignment");
        _indices.push_back( (unsigned int) assign_labels[xi.k]);
      }  
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
  ConvergeState KMedoids::calculateMedoids()
  {
    std::vector<Eigen::Isometry3d> u_old;
    u_old = u_;

    for(int k = 0; k < k_; ++k)
    {
      Eigen::Vector3d num(0, 0, 0);
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

      if(den > 0)
      {
       num /= (double) den;
       Eigen::Isometry3d u_old; u_old = u_[k];
       u_[k] = Exp(u_[k], num);
      }
    } // for k
 
    
    // Check convergence
    double u_diff, sum_diff;
    sum_diff = 0;
    
    std::string str;
    for(int k = 0; k < k_; ++k)
    {
      u_diff = d(u_[k], u_old[k]); //(u_[k] - u_old[k]).norm();

      if(std::isnan(u_diff))
        return IS_NAN;
        
      sum_diff += u_diff;
    }

    return (sum_diff / (double)k_) < medoids_thresh_ ? CONVERGED : NON_CONVERGED;

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
