#pragma once


#include <kdl/treefksolver.hpp>

namespace robot_unit
{
/**
 * @class TreeFkSolverPos_recursive
 */
class TreeFkSolverPos_recursive : public KDL::TreeFkSolverPos
{
public:
  TreeFkSolverPos_recursive(const KDL::Tree &_tree);
  ~TreeFkSolverPos_recursive() {}


  virtual int JntToCart(const KDL::JntArray& _q_in,
                        KDL::Frame& _p_out,
                        std::string segmentName);

  int JntToCart(const KDL::JntArray& _q_in,
                std::map<std::string, KDL::Frame> &_p_out);

  int getSegmentsRecursiveOrder(std::vector<std::string> &_links);

  void updateTfRefRoot(const KDL::Frame &_Tfx);

private:
  KDL::Frame recursiveFk(const KDL::JntArray& _q_in,
                         const KDL::SegmentMap::const_iterator& _it);

  int recursiveFkFull(const KDL::JntArray& _q_in,
                      std::map<std::string, KDL::Frame> &_p_out,
                      KDL::SegmentMap::const_iterator& _it,
                      const KDL::Frame &_parent_frame);

  int getSegmentsRecursive(KDL::SegmentMap::const_iterator& _it,
                           std::vector<std::string> &_links);

  const KDL::Tree tree_;
  KDL::Frame Tf_ref_root_;
};

} // namespace robot_unit
