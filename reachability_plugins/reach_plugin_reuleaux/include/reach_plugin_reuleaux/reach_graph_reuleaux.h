#pragma once

/**
 * @file reach_graph_reuleaux.h
 */
#include <reachability_description/reach_graph.h>

/**
 * @class ReachGraphReuleaux
 */
namespace reachability_description
{

class ReachGraphReuleaux : public reachability_description::ReachGraph {

 public:
  ReachGraphReuleaux();
  ~ReachGraphReuleaux();

  void generateSamples(const double &_xi, const double &_yi, const double &_zi, std::vector<Eigen::Isometry3d> &_frames) override;
  bool calculateMetric(reachability_msgs::msg::ReachData &_rdata, const std::shared_ptr<KDL::ChainJntToJacSolver> &_jac_solver) override;

  sensor_msgs::msg::PointCloud2 debugSamples(int _xi, int _yi, int _zi);


  protected:

  void createSphereSamplesVoxel(const double &_x, 
                                const double &_y, 
                                const double &z,
                                std::vector<Eigen::Isometry3d> &_frames) const;

  void createTesseractSamples(const double &_x, 
                              const double &_y, 
                              const double &_z,
                              std::vector<Eigen::Isometry3d> &_frames) const;


};

} // namespace reachability_description
