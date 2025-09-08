

#include <reachability_description/gmm_s2.h>
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
      // Normalize for good measure
      u_m_new.normalize();
       
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
     
//     RCLCPP_INFO(rclcpp::get_logger("GMMS2"), "Log between  %f %f %f and %f %f %f is: %f", 
//                 _x.x(), _x.y(), _x.z(), _y.x(), _y.y(), _y.z(), u);

     
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
   
   return y;
  }

} // namespace s2
