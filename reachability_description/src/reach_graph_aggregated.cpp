/**
 * @file reach_graph_aggregated.cpp 
 */
#include <reachability_description/reach_graph_aggregated.h>
#include <reachability_description/reach_utilities.h>

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
std::vector<PlaceSol> ReachGraphAggregated::getSolutions(const std::vector<Eigen::Isometry3d> &_Tgs, 
                                                         const std::vector<InvData> &_candidates,
                                                         const int &_desired_num_solutions)
{
   return solvePCA(_Tgs, _candidates, _desired_num_solutions);
}                                        

/**
 * @function fillIndex 
 */
void ReachGraphAggregated::fillIndex(const std::vector<InvData> &_candidates,
                                     std::shared_ptr<cv::flann::Index> &_index)
{
   cv::flann::KDTreeIndexParams params(1);

   cv::Mat data(_candidates.size(), 3, CV_32FC1);
   for(int i = 0; i < _candidates.size(); ++i)
   {
    Eigen::Vector3d p;
    p = _candidates[i].place.Twb.translation();
    data.at<float>(i, 0) = (float)p(0);
    data.at<float>(i, 1) = (float)p(1);
    data.at<float>(i, 2) = (float)p(2);   
   }     

   _index.reset(new cv::flann::Index(data, params));
}

/**
 * @function getNN 
 */
int ReachGraphAggregated::getNN(const double &_x, const double &_y, const double &_z,
          const std::shared_ptr<cv::flann::Index> &index_ptr,
          std::vector<int> &_inds,
          const float &_nn_radius,
          const int &_max_neighbors)
{
    // Reset
   _inds.clear();

   // Calculate
   cv::Mat indices, dists;

    cv::Mat query(1, 3, CV_32FC1);
    query.at<float>(0,0) = _x;
    query.at<float>(0,1) = _y;
    query.at<float>(0,2) = _z;

    int num_nn = index_ptr->radiusSearch(query, 
                            indices, dists, 
                            _nn_radius, _max_neighbors, 
                            cv::flann::SearchParams(32));

    for(int i = 0; i < num_nn; ++i)
        _inds.push_back(indices.at<int>(i));

    return num_nn;
}

/**
 * @function solveSimpleProjection 
 */
std::vector<PlaceSol> ReachGraphAggregated::solvePCA(const std::vector<Eigen::Isometry3d> &_Tgs, 
                                                     const std::vector<InvData> &_candidates,
                                                     const int &_num_clusters)
{
    float nn_radius = 0.05*0.05;
    int max_neighbors = 20;

   //1. Get centers
   cv::Mat points(_candidates.size(), 1, CV_32FC3);
   
   cv::Mat labels;
   std::vector<cv::Point3f> centers;

   // 2. Fill points
   for(int i = 0; i < _candidates.size(); ++i)
   {
    Eigen::Vector3d p;
    p = _candidates[i].place.Twb.translation();
    points.at<cv::Vec3f>(i) = cv::Vec3f( (float)p(0), (float)p(1), (float)p(2));
   }     

   cv::kmeans(points, _num_clusters, labels,
   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
   3, cv::KMEANS_PP_CENTERS, centers);

   // Get close points to the centers
   std::shared_ptr<cv::flann::Index> index_ptr;
   fillIndex(_candidates, index_ptr);

   std::vector<InvData> id;
   std::vector< std::vector<InvData> > id_nn;
   for(int i = 0; i < centers.size(); ++i)
   {
    std::vector<int> indices;
    if( getNN(centers[i].x, centers[i].y, centers[i].z,
              index_ptr, indices, nn_radius, max_neighbors) == 0)
        continue;

    std::vector<InvData> id_nn_i;
    for(int j = 0; j < indices.size(); ++j)
      id_nn_i.push_back( _candidates[indices[j]] );

    id.push_back(_candidates[indices[0]]);
    id_nn.push_back(id_nn_i);
 
   } // for centers


   // 2. See if there are solutions
   std::vector<PlaceSol> sols;
   for(int i = 0; i < id.size(); ++i)
   {
    if(simpleYawSearch(_Tgs, id[i]))
        sols.push_back(id[i].place);
   }

    return sols;
}                                                         

/**
 * @function simpleYawSearch 
 */
bool ReachGraphAggregated::simpleYawSearch(const std::vector<Eigen::Isometry3d> &_Tgs,
                        InvData &_candidate)
{
   _candidate.place.Twb = _candidate.place.project();

   KDL::JntArray max_q;
   double max_metric = 0;
   Eigen::Matrix3d max_rot;
   
   std::vector<PlaceSol> valid_rotations;
   for(int i = 0; i < 12; ++i)
   {
      bool found_solution_task_step = true;
      PlaceSol ps;
      ps.Twb = _candidate.place.Twb;
      Eigen::Matrix3d m;
      m = Eigen::AngleAxisd(-M_PI + i*30.0/180.0*M_PI, Eigen::Vector3d(0,0,1));
      ps.Twb.linear() = m;

      for(auto Tgi : _Tgs)
      {
         KDL::JntArray q_init, q_out;
         q_init = vectorToJntArray(_candidate.sample.q);

         KDL::Twist bounds = KDL::Twist::Zero();
         KDL::Frame p_in;
         tf2::transformEigenToKDL(ps.Twb.inverse()*Tgi, p_in);
         rd_->getIKSolver(chain_group_)->CartToJnt(q_init, p_in, q_out, bounds);

         std::vector<KDL::JntArray> all_sols;
         if(!rd_->getIKSolver(chain_group_)->getSolutions(all_sols))
         {
            found_solution_task_step = false;
            break;
         }

         std::vector<KDL::JntArray> coll_free_sols;
         for(int j = 0; j < all_sols.size(); ++j) 
         {
           // Check if solution is collision free
           if(!rd_->isSelfColliding( jntArrayToMsg(all_sols[j], chain_info_)))
             coll_free_sols.push_back(all_sols[j]);
         }

         // No solutions collision-free
         if(coll_free_sols.empty())
         {
            found_solution_task_step = false;
            break;
         }
         else
         {
            // Store
            ps.q.push_back(coll_free_sols[0]);
         }


      } // Tgi

      if(found_solution_task_step)
      {
        valid_rotations.push_back(ps);
      }

    } // for i

    /*
           double metric = jointRangeMetric(all_sols[j]);
           //double metric = jointDistancePreferred();
           if( metric  > max_metric)
           {
             max_metric = metric;
             max_rot = m;
             max_q = q_out;
           }*/
//    if(max_metric > 0)
//    {
       RCLCPP_INFO(rclcpp::get_logger("reach_graph"), "Valid rotations: %lu ", valid_rotations.size());
    if(valid_rotations.size() > 0)
    {  
        _candidate.place = valid_rotations[0];
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
        ps.q.push_back(q_out);
        sols.push_back(ps);
    }

   }

   return sols;
}   

/**
 * @function getCandidates
 * @brief  
 */
bool ReachGraphAggregated::getCandidates(const std::vector<Eigen::Isometry3d> &_Tgs, 
                                        std::vector<InvData> &_candidates,
                                        std::vector<InvData> &_best_candidates)
{
    if(_Tgs.empty())
        return false;

    double z_thresh = rd_->getReachGraph(chain_group_)->getResolution();
    double angle_thresh = 30.0*M_PI/180.0;
    int samples_thresh = (int)(0.5* (double) max_voxel_samples_);

    // Reset
    _candidates.clear();
    _best_candidates.clear();

    std::vector<std::vector<InvData>> inv_data(_Tgs.size());
    std::vector<std::vector<InvData>> best_inv_data(_Tgs.size());
    for(int i = 0; i < _Tgs.size(); ++i)
    {
       if(!getCandidates(_Tgs[i], inv_data[i], best_inv_data[i], samples_thresh, z_thresh, angle_thresh))
            return false;

        RCLCPP_INFO(rclcpp::get_logger("reach_graph"), "CANDIDATE [%d]: Size: %lu - %lu", i, inv_data[i].size(), best_inv_data[i].size() );        
    }

    // Get intersection
    std::vector<InvData> culled_candidates = best_inv_data[0];

        RCLCPP_INFO(rclcpp::get_logger("reach_graph"), "Culled candidates start: %ld", culled_candidates.size());

    for(int i = 1; i < best_inv_data.size(); ++i)
      intersect_candidates(culled_candidates, best_inv_data[i]);

        RCLCPP_INFO(rclcpp::get_logger("reach_graph"), "Culled candidates after: %ld", culled_candidates.size());

    // Just put everything together
    /*
    for(int i = 0; i < inv_data.size(); ++i )
    {
        _candidates.insert(_candidates.end(), inv_data[i].begin(), inv_data[i].end());
        _best_candidates.insert(_best_candidates.end(), best_inv_data[i].begin(), best_inv_data[i].end());
    }*/

    _best_candidates = culled_candidates;
    return !_best_candidates.empty();
}

/**
 * @function intersect_candidates 
 */
bool ReachGraphAggregated::intersect_candidates(std::vector<InvData> &_culled_candidates,
                          const std::vector<InvData> &_additional_candidates)
{
    RCLCPP_INFO(rclcpp::get_logger("reach_graph"), "Culled candidates: %ld and additional candidates: %ld", 
    _culled_candidates.size(), _additional_candidates.size());

   float nn_radius = 0.05;
   int max_neighbors = 20;
   std::vector<InvData> intersect_candidates;

   std::shared_ptr<cv::flann::Index> index_ptr;
   fillIndex(_additional_candidates, index_ptr);

   for(auto ci : _culled_candidates)
   {
    std::vector<int> indices;
    if( getNN(ci.place.Twb.translation()(0), 
              ci.place.Twb.translation()(1),
              ci.place.Twb.translation()(2),
              index_ptr, indices, nn_radius, max_neighbors) == 0)
        continue;

     intersect_candidates.push_back(ci);
   }
   RCLCPP_INFO(rclcpp::get_logger("reach_graph"), "Intersect  candidates after: %lu ", intersect_candidates.size());

   _culled_candidates = intersect_candidates;

   return (!_culled_candidates.empty());
}

/**
 * @function getCandidates 
 * @brief Get samples that allow to reach this transform with a floor constraint
 */
bool ReachGraphAggregated::getCandidates(const Eigen::Isometry3d &_Tg,
                        std::vector<InvData> &_candidates,
                        std::vector<InvData> &_best_candidates,
                        const int &_samples_thresh,
                        const double &_z_thresh,
                        const double &_angle_thresh)
{                   
   std::vector< std::vector<Sample> >::iterator s_iter;

   for(s_iter = samples_.begin(); s_iter != samples_.end(); ++s_iter)
   {
      if(s_iter->size() < _samples_thresh)
        continue;

      std::vector<Sample>::iterator iti;
      for(iti = s_iter->begin(); iti != s_iter->end(); ++iti)
      {
         PlaceSol ci;
         ci.Twb = _Tg* (iti->Tf_ee_inv);

         InvData id;
         id.place = ci;
         id.sample = *iti;
           
         if( checkConstraints(ci.Twb, _z_thresh, _angle_thresh) )
           _best_candidates.push_back(id);
         else
           _candidates.push_back(id);
                      

       } // for iti

    } // for s_iter 


   return !_best_candidates.empty();    
}


bool ReachGraphAggregated::checkConstraints( const Eigen::Isometry3d &_Twb, const double &_z_max, const double &_z_angle_max ) {

  if(fabs(_Twb.translation()(2)) < _z_max)
  { 
     double z_angle;
     z_angle = (_Twb.linear().col(2)).dot( Eigen::Vector3d(0,0,1));


     if( fabs( acos(z_angle) ) < _z_angle_max )
       return true;
     else
       return false;
  }
  
  else
    return false;
}

