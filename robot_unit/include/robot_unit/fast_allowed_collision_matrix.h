#pragma once

#include <map>
#include <vector>
#include <string>

namespace robot_unit
{

class AllowedCollisionMatrix
{
public:
  AllowedCollisionMatrix();
  AllowedCollisionMatrix(const std::vector<std::string>&_links,
                         bool _allowed = false);
  bool setEntry(const std::string &_link_1,
                const std::string &_link_2,
                bool _allowed);
  bool getEntry(const std::string& name1,
                const std::string& name2) const;

protected:
  std::map<std::string, std::map<std::string, bool> > entries_;
};

} // namespace plummrs
