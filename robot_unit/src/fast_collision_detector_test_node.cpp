/**
 * @file fast_collision_detector_test_node.cpp
 */
#include <ros/ros.h>
#include <fast_collision_detector/fast_collision_detector.h>

/**
 * @function main
 */
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "fast_collision_detector_test_node");
  plummrs::FastCollisionDetector fcd;

  // Add bumble and honey
  if (!fcd.addRobot("bumble"))
    return 0;

  if (!fcd.addRobot("honey"))
    return 0;

  // Check
  while (ros::ok())
  {
    ros::spinOnce();
    fcd.checkCollisions();
  }

  return 1;
}
