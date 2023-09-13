/**
 * @file reach_graph_aggregated.cpp 
 */
#include <reachability_description/reach_graph_aggregated.h>
#include <reachability_description/reach_utilities.h>

#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>

double getYaw(const Eigen::Isometry3d &_Tfx)
{
  Eigen::Matrix3d m; m = _Tfx.linear();
  Eigen::Vector3d ypr; ypr = m.eulerAngles(2,1,0);
  return ypr(0);
}


/**
 * @brief Constructor 
 */
ReachGraphAggregated::ReachGraphAggregated()
{

}

/**
 * @function reset 
 */
void ReachGraphAggregated::reset()
{
    samples_.clear();

    min_voxel_samples_ = 1000;
    max_voxel_samples_ = 0;
}

/**
 * @function fillFromGraph 
 */
bool ReachGraphAggregated::fillFromGraph( const std::shared_ptr<reachability_description::ReachabilityDescription> &_rd, 
                                          const std::string &_chain_group)
{
    chain_group_ = _chain_group;
    rd_ = _rd;
    std::shared_ptr<reachability_description::ReachGraph> rg = rd_->getReachGraph(_chain_group);

    if(rg == nullptr)
        return false;

    if(!rd_->getChainInfo(_chain_group, chain_info_))
        return false;

    // Clean up first
    this->reset();

    for(int i = 0; i < rg->getNumPoints(); ++i)
    {
        reachability_msgs::msg::ReachData rdi = rg->getState(i);
        std::vector<Sample> samples_i;

        for(auto ri : rdi.samples)
        {
            Sample si;
            tf2::fromMsg(ri.pose, si.Tf_ee);
            si.Tf_ee_inv = si.Tf_ee.inverse();
            si.q = ri.best_config;

            samples_i.push_back(si);
        } // for j

        samples_.push_back(samples_i);

        if(rdi.samples.size() < min_voxel_samples_)
            min_voxel_samples_ = rdi.samples.size();

        if(rdi.samples.size() > max_voxel_samples_)
            max_voxel_samples_ = rdi.samples.size();
        
    } // for i

    // Fill chain data
    std::vector<std::pair<double, double>> joint_limits;
    joint_limits = rd_->getJointLimits(chain_group_);
    for(auto ji : joint_limits)
    {
        joint_lower_lim_.push_back(ji.first);
        joint_upper_lim_.push_back(ji.second);
    }

   printf("After filling: min: %d max: %d \n", min_voxel_samples_, max_voxel_samples_);
   return true;
}

/**
 * @function getSolutions 
 */
std::vector<PlaceSol> ReachGraphAggregated::getSolutions(const Eigen::Isometry3d &_Tg, 
                                                         const std::vector<PlaceSol> &_candidates,
                                                         const std::vector<Sample> &_samples,
                                                         const int &_desired_num_solutions)
{
   //return solveSimpleProjection(_Tg, _candidates, _samples);
   return solvePCA(_Tg, _candidates, _samples, _desired_num_solutions);
}                                        

/**
 * @function solveSimpleProjection 
 */
std::vector<PlaceSol> ReachGraphAggregated::solvePCA(const Eigen::Isometry3d &_Tg, 
                                                     const std::vector<PlaceSol> &_candidates,
                                                     const std::vector<Sample> &_samples,
                                                     const int &_num_clusters)
{
   //1. Get centers
   int max_neighbors = 20;
   cv::Mat points(_candidates.size(), 1, CV_32FC3);
   cv::Mat data(_candidates.size(), 3, CV_32FC1);
   cv::Mat labels;
   std::vector<cv::Point3f> centers;

   // 2. Fill points
   for(int i = 0; i < _candidates.size(); ++i)
   {
    Eigen::Vector3d p;
    p = _candidates[i].Twb.translation();
    points.at<cv::Vec3f>(i) = cv::Vec3f( (float)p(0), (float)p(1), (float)p(2));
    data.at<float>(i, 0) = (float)p(0);
    data.at<float>(i, 1) = (float)p(1);
    data.at<float>(i, 2) = (float)p(2);   
   }     

   cv::kmeans(points, _num_clusters, labels,
   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
   3, cv::KMEANS_PP_CENTERS, centers);

   // Get close points to the centers
   cv::Mat indices, dists;
   float radius = 0.05*0.05;
   cv::flann::KDTreeIndexParams params(1);
   cv::flann::Index index(data, params);


    std::vector<PlaceSol> cis;
    std::vector<Sample> sis;

   std::vector< std::vector<PlaceSol> > cis_nn;
   std::vector< std::vector<Sample> > sis_nn;
   for(int i = 0; i < centers.size(); ++i)
   {
    RCLCPP_INFO(rclcpp::get_logger("reach_graph"), "Center[%d]: %f %f %f", i, centers[i].x, centers[i].y, centers[i].z );
    cv::Mat query(1, 3, CV_32FC1);
    query.at<float>(0,0) = centers[i].x;
    query.at<float>(0,1) = centers[i].y;
    query.at<float>(0,2) = centers[i].z;

    int num = index.radiusSearch(query, indices, dists, radius, max_neighbors, cv::flann::SearchParams(32));


    std::vector<PlaceSol> cis_nn_i;
    std::vector<Sample> sis_nn_i;
    for(int i = 0; i < num; ++i)
    {
        int ind = indices.at<int>(i);
        cv::Vec3f p = points.at<cv::Vec3f>(ind); // dists.at<float>(i)
        
        cis_nn_i.push_back( _candidates[ind] );
        sis_nn_i.push_back( _samples[ind] );
    } // for k closest

    if(num > 0)
    {
        cis.push_back(_candidates[indices.at<int>(0)]);
        sis.push_back(_samples[indices.at<int>(0)]);
        cis_nn.push_back(cis_nn_i);
        sis_nn.push_back(sis_nn_i);
    }

   } // for centers


   // 2. See if there are solutions
   std::vector<PlaceSol> sols;
   for(int i = 0; i < cis.size(); ++i)
   {
    if(simpleYawSearch(_Tg, cis[i], sis[i]))
        sols.push_back(cis[i]);

   }

    return sols;
}                                                         

/**
 * @function simpleYawSearch 
 */
bool ReachGraphAggregated::simpleYawSearch(const Eigen::Isometry3d &_Tg,
                     PlaceSol &_candidate, 
                     const Sample &_sample)
{
   _candidate.Twb = _candidate.project();

   KDL::JntArray max_q;
   double max_metric = 0;
   Eigen::Matrix3d max_rot;

   for(int i = 0; i < 12; ++i)
   {
      Eigen::Matrix3d m;
      m = Eigen::AngleAxisd(-M_PI + i*30.0/180.0*M_PI, Eigen::Vector3d(0,0,1));

      _candidate.Twb.linear() = m;

      // IK
      KDL::JntArray q_init, q_out;
      q_init = vectorToJntArray(_sample.q);

      KDL::Twist bounds = KDL::Twist::Zero();
      KDL::Frame p_in;
      tf2::transformEigenToKDL(_candidate.Twb.inverse()*_Tg, p_in);
      int ik_sol = rd_->getIKSolver(chain_group_)->CartToJnt(q_init, p_in, q_out, bounds);
      
      std::vector<KDL::JntArray> all_sols;
      if(!rd_->getIKSolver(chain_group_)->getSolutions(all_sols))
        continue;
        
       RCLCPP_INFO(rclcpp::get_logger("reach_graph"), "All sols: %lu ", all_sols.size());
 
       for(int j = 0; j < all_sols.size(); ++j) 
       {
        // Check if solution is collision free
        if(rd_->isSelfColliding( jntArrayToMsg(all_sols[j], chain_info_)))
          continue;

        double metric = jointRangeMetric(all_sols[j]);
        //double metric = jointDistancePreferred();
        if( metric  > max_metric)
        {
            max_metric = metric;
            max_rot = m;
            max_q = q_out;
        }
      } // for j

    } // for i
    if(max_metric > 0)
    {
        _candidate.Twb.linear() = max_rot;
        _candidate.q = max_q;
        return true;
    }
    else
        return false;
}

/**
 * @function jointRangeMetric 
 */
double ReachGraphAggregated::jointRangeMetric(const KDL::JntArray &_q)
{
    double res = 1;
    double factor;
    double range;
    for(int i = 0; i < joint_upper_lim_.size(); ++i)
    {
        range = joint_upper_lim_[i] - joint_lower_lim_[i];
        factor = (_q(i) - joint_lower_lim_[i])*(joint_upper_lim_[i] - _q(i))/(range*range);
        res *= factor;
    }

    return res;
}

/**
 * @function solveSimpleProjection 
 */
std::vector<PlaceSol> ReachGraphAggregated::solveSimpleProjection(const Eigen::Isometry3d &_Tg, 
                                                         const std::vector<PlaceSol> &_candidates,
                                                         const std::vector<Sample> &_samples)
{
   // 1. Transform _points rotation into a planar  rotation 
   std::vector<Eigen::Isometry3d> planars;
   for(int i = 0; i < _candidates.size(); ++i)
   planars.push_back(_candidates[i].project(0));

   // 2. See if there are solutions
   std::vector<PlaceSol> sols;
   for(int i = 0; i < planars.size(); ++i)
   {
    KDL::JntArray q_init, q_out;
    q_init = vectorToJntArray(_samples[i].q);

    KDL::Twist bounds = KDL::Twist::Zero();
    KDL::Frame p_in;
    tf2::transformEigenToKDL(planars[i].inverse()*_Tg, p_in);
    int ik_sol = rd_->getIKSolver(chain_group_)->CartToJnt(q_init, p_in, q_out, bounds);
    if(ik_sol > 0)
    {
        PlaceSol ps;
        ps.Twb = planars[i];
        ps.q = q_out;
        sols.push_back(ps);
    }

   }

   return sols;
}   

/**
 * @function getCandidates
 */
bool ReachGraphAggregated::getCandidates(const Eigen::Isometry3d &_Tg, 
                                        std::vector<PlaceSol> &_candidates,
                                        std::vector<Sample> &_samples,
                                        int &_best_candidates)
{
    double thresh = 0.05;
    int num_voxels = 0;
    int num_samples_acc = 0;
    int maximum_samples = 0;
    double angle_thresh = 30.0*M_PI/180.0;

    // Reset
    _candidates.clear();
    _samples.clear();

    int samples_thresh = (int)(0.5* (double) max_voxel_samples_);

    std::vector<PlaceSol> best_candidates;
    std::vector<Sample> best_samples;
    std::vector< std::vector<Sample> >::iterator s_iter;

    for(s_iter = samples_.begin(); s_iter != samples_.end(); ++s_iter)
    {
        if(s_iter->size() < samples_thresh)
            continue;

       int num_samples_within_thresh = 0;
       std::vector<Sample>::iterator iti;
       for(iti = s_iter->begin(); iti != s_iter->end(); ++iti)
       {
          PlaceSol ci;

          // Behind the robot // TESTING 
          //if(iti->Tf_ee.translation()(0) < 0)
          //  continue;

          ci.Twb = _Tg* (iti->Tf_ee_inv);
           
          if(fabs(ci.Twb.translation()(2)) < thresh)
          { 
             num_samples_within_thresh++;
            double z_angle;
            z_angle = (ci.Twb.linear().col(2)).dot( Eigen::Vector3d(0,0,1));
            if( fabs( acos(z_angle) ) < angle_thresh )
            {
                best_samples.push_back(*iti);
                best_candidates.push_back(ci);
            } else
            {
            _samples.push_back(*iti);
            _candidates.push_back(ci);
            }

          }       

       } // for 


      if(num_samples_within_thresh > 0)
      {
        num_samples_acc += num_samples_within_thresh;

        num_voxels++;
        if(num_samples_within_thresh > maximum_samples)
        {
            maximum_samples = num_samples_within_thresh;
        }
      }

    } // for 

        // Add best points at the end
        _best_candidates = best_candidates.size();
        _candidates.insert(_candidates.end(), best_candidates.begin(), best_candidates.end());
        _samples.insert(_samples.end(), best_samples.begin(), best_samples.end());

    RCLCPP_INFO(rclcpp::get_logger("reach_graph"), "# voxels with sols: %d. Num total samples: %ld, strict: %d (%f) Maximum samples with sol in a voxel: %d.\n", 
            num_voxels, _candidates.size(), _best_candidates, (double) _best_candidates/(double)_candidates.size() , maximum_samples);

    return true;
}