#include <robot_unit/fast_allowed_collision_matrix.h>

using namespace robot_unit;

AllowedCollisionMatrix::AllowedCollisionMatrix()
{

}

AllowedCollisionMatrix::AllowedCollisionMatrix(const std::vector<std::string>&_links,
    bool _allowed)
{
  entries_.clear();
  for (std::size_t i = 0; i < _links.size(); ++i)
    for (std::size_t j = i; j < _links.size(); ++j)
      setEntry(_links[i], _links[j], _allowed);
}

bool AllowedCollisionMatrix::setEntry(const std::string &_link_1,
                                      const std::string &_link_2,
                                      bool _allowed)
{
  entries_[_link_1][_link_2] = entries_[_link_2][_link_1] = _allowed;
  return true;
}

bool AllowedCollisionMatrix::getEntry(const std::string& name1,
                                      const std::string& name2) const
{
  auto it1 = entries_.find(name1);
  if (it1 == entries_.end())
    return false;

  auto it2 = it1->second.find(name2);
  if (it2 == it1->second.end())
    return false;

  //  allowed_collision = it2->second;
  return it2->second;
}
