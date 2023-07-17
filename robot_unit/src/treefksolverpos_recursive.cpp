#include <robot_unit/treefksolverpos_recursive.hpp>
#include <iostream>
#include <Eigen/Geometry>

namespace robot_unit
{

TreeFkSolverPos_recursive::TreeFkSolverPos_recursive(const KDL::Tree &_tree) :
  tree_(_tree),
  Tf_ref_root_(KDL::Frame::Identity())
{

}

/**
 * @brief NOTE! You have to call recursive JntToCart for this update to take effect
 * @brief in the transforms you obtain
 */
void TreeFkSolverPos_recursive::updateTfRefRoot(const KDL::Frame &_Tfx)
{
  Tf_ref_root_ = _Tfx;
}

int TreeFkSolverPos_recursive::JntToCart(const KDL::JntArray& _q_in,
    KDL::Frame& _p_out,
    std::string _segmentName)
{
  KDL::SegmentMap::const_iterator it = tree_.getSegment(_segmentName);
  if (_q_in.rows() != tree_.getNrOfJoints())
    return -1;
  else if (it == tree_.getSegments().end())
    return -2;
  else
  {
    _p_out = recursiveFk(_q_in, it);
    return 0;
  }
}

KDL::Frame TreeFkSolverPos_recursive::recursiveFk(const KDL::JntArray& _q_in,
    const KDL::SegmentMap::const_iterator& _it)
{
  const KDL::TreeElementType& current_elem = _it->second;
  KDL::Frame current_frame = GetTreeElementSegment(current_elem).pose(_q_in(GetTreeElementQNr(current_elem)));

  KDL::SegmentMap::const_iterator root_iter = tree_.getRootSegment();
  if (_it == root_iter)
    return current_frame;
  else
  {
    KDL::SegmentMap::const_iterator parent_it = GetTreeElementParent(current_elem);
    return recursiveFk(_q_in, parent_it) * current_frame;
  }
}

int TreeFkSolverPos_recursive::JntToCart(const KDL::JntArray& _q_in,
    std::map<std::string, KDL::Frame> &_p_out)
{
  if (_q_in.rows() != tree_.getNrOfJoints())
    return -1;

  KDL::SegmentMap::const_iterator root_iter = tree_.getRootSegment();

  return recursiveFkFull(_q_in, _p_out, root_iter, Tf_ref_root_);
}

int TreeFkSolverPos_recursive::recursiveFkFull(const KDL::JntArray& _q_in,
    std::map<std::string, KDL::Frame> &_p_out,
    KDL::SegmentMap::const_iterator& _it,
    const KDL::Frame &_parent_frame)
{

  const KDL::TreeElementType& current_elem = _it->second;
  KDL::Frame current_frame = GetTreeElementSegment(current_elem).pose(_q_in(GetTreeElementQNr(current_elem)));

  // Store frame = parent*local_frame
  current_frame = _parent_frame * current_frame;
  _p_out[_it->first] = current_frame;


  // If no more children, return successfully, nothing to update
  if (_it->second.children.empty())
    return 1;

  for (auto ci : _it->second.children)
  {
    if (recursiveFkFull(_q_in, _p_out, ci, current_frame) != 1)
    {
      printf("\t * Error calculating the recursive FK of %s \n", ci->first.c_str());
      return -2;
    }
  }

  return 1;
}



int TreeFkSolverPos_recursive::getSegmentsRecursiveOrder(std::vector<std::string> &_links)
{
  _links.clear();
  KDL::SegmentMap::const_iterator root_iter = tree_.getRootSegment();
  return getSegmentsRecursive(root_iter, _links);

}

int TreeFkSolverPos_recursive::getSegmentsRecursive(KDL::SegmentMap::const_iterator& _it,
    std::vector<std::string> &_links)
{
  const KDL::TreeElementType& current_elem = _it->second;

  _links.push_back(_it->first);

  // If no more children, return successfully, nothing to update
  if (_it->second.children.empty())
    return 1;

  for (auto ci : _it->second.children)
  {
    if (getSegmentsRecursive(ci, _links) != 1)
    {
      printf("\t * Error calculating the recursive ordering of %s \n", ci->first.c_str());
      return -2;
    }
  }

  return 1;
}

} // namespace plummrs
