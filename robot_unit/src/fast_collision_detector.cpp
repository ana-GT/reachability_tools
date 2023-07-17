#include <fast_collision_detector/fast_collision_detector.h>
#include <fcl/geometry/bvh/BVH_model.h>

using namespace plummrs;

FastCollisionDetector::FastCollisionDetector()
{

}

bool FastCollisionDetector::addRobot(std::string _robot)
{
  if (robots_.find(_robot) != robots_.end())
  {
    ROS_ERROR("Robot %s already exists in robots_ structure", _robot.c_str());
    return false;
  }

  RobotCollisionObjectPtr ri = RobotCollisionObjectPtr(new RobotCollisionObject());

  if (ri->init(_robot))
    robots_[_robot] = ri;


  return true;
}


bool FastCollisionDetector::checkCollisions()
{
  return true;
}
