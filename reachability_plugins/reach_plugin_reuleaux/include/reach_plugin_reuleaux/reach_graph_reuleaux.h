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

  void generateSamples(const int &_xi, const int &_yi, const int &_zi, std::vector<Eigen::Isometry3d> &_frames) override;


  sensor_msgs::msg::PointCloud2 debugSamples(int _xi, int _yi, int _zi);


  protected:

  void createSphereSamplesVoxel(const int &_xi, 
                                const int &_yi, 
                                const int &zi,
                                std::vector<Eigen::Isometry3d> &_frames) const;

  void createTesseractSamples(const int &_xi, 
                              const int &_yi, 
                              const int &_zi,
                              std::vector<Eigen::Isometry3d> &_frames) const;


};

} // namespace reachability_description
