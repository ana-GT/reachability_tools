#pragma once

#include <reachability_description/reach_data.h>
#include <reachability_description/reachability_description.h>
#include <Eigen/Geometry>

struct Sample
{
  Eigen::Isometry3d Tf_ee;
  Eigen::Isometry3d Tf_ee_inv;
  std::vector<double> q;
};

double getYaw(const Eigen::Isometry3d &_Tfx);

struct PlaceSol
{
   Eigen::Isometry3d Twb;
   KDL::JntArray q;

   // Keep x and y, yaw, project to a plane in Z
   Eigen::Isometry3d project(const double &_z = 0) const
   {
    double yaw = getYaw(Twb);

    Eigen::Isometry3d proj;
    proj.setIdentity();
    proj.translation() = Twb.translation();
    proj.translation()(2) = _z;
    proj.linear() = Eigen::Matrix3d( Eigen::AngleAxis( yaw, Eigen::Vector3d(0,0,1)) );
    return proj;
   }

   static  Eigen::Isometry3d projectNN( const std::vector<PlaceSol> &_cis_nn )
   {
      Eigen::Isometry3d Tf_proj;
      Tf_proj.setIdentity();

      // Translation
      Eigen::Vector3d p; p << 0, 0, 0;
      for(int i = 0; i < _cis_nn.size(); ++i)
        p += _cis_nn[i].Twb.translation();

      p /= (double) _cis_nn.size();
      Tf_proj.translation() << p(0), p(1), 0;

      // Rotation
      double yaw = 0;
      for(int i = 0; i < _cis_nn.size(); ++i)
        yaw += getYaw(_cis_nn[i].Twb);
      
      yaw /= (double) _cis_nn.size();

      Tf_proj.linear() = Eigen::Matrix3d( Eigen::AngleAxis( yaw, Eigen::Vector3d(0,0,1)) );


      return Tf_proj;
   }

};

/**
 * @class ReachGraphAggregated 
 */
class ReachGraphAggregated
{
  public:
  ReachGraphAggregated();
  void reset();
  bool fillFromGraph(const std::shared_ptr<reachability_description::ReachabilityDescription> &_rd, 
                     const std::string &_chain_group);
  bool getCandidates(const Eigen::Isometry3d &_Tg, 
                    std::vector<PlaceSol> &_candidates,
                    std::vector<Sample> &_samples,
                    int &_best_candidates);
  std::vector<PlaceSol> getSolutions(const Eigen::Isometry3d &_Tg, 
                                     const std::vector<PlaceSol> &_candidates,
                                     const std::vector<Sample> &_samples,
                                     const int &_desired_num_solutions);     


  // Solutions
  std::vector<PlaceSol> solveSimpleProjection(const Eigen::Isometry3d &_Tg, 
                                              const std::vector<PlaceSol> &_candidates,
                                              const std::vector<Sample> &_samples);                                                   

  std::vector<PlaceSol> solvePCA(const Eigen::Isometry3d &_Tg, 
                                 const std::vector<PlaceSol> &_candidates,
                                 const std::vector<Sample> &_samples,
                                 const int &_num_clusters);

  // Helpers
  bool simpleYawSearch(const Eigen::Isometry3d &_Tg,
                     PlaceSol &_candidate, 
                     const Sample &_sample);
  double jointRangeMetric(const KDL::JntArray &_q);                   

  protected:

  std::vector< std::vector<Sample> > samples_;

  int min_voxel_samples_;
  int max_voxel_samples_;
  std::string chain_group_;
  reachability_msgs::msg::ChainInfo chain_info_;
  std::vector<double> joint_lower_lim_;
  std::vector<double> joint_upper_lim_;

 std::shared_ptr<reachability_description::ReachabilityDescription> rd_;



};