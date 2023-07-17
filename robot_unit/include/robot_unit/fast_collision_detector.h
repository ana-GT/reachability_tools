
#pragma once

#include <ros/ros.h>
#include <fast_collision_detector/fast_robot_collision_object.h>

namespace plummrs
{

/**
 * @class FastCollisionDetector
 */
class FastCollisionDetector
{

public:
  FastCollisionDetector();
  bool addRobot(std::string _robot);
  bool checkCollisions();

protected:
  std::map<std::string, RobotCollisionObjectPtr> robots_;

};


} // namespace plummrs
