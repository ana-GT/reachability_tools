#pragma once

#include <Eigen/Geometry>
#include <robot_unit/robot_entity.h>

struct GoalInfo {
    std::string group;
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
};

struct CostData {
    robot_entity::RobotEntity re;
    GoalInfo goal;
};


double cost_function(unsigned n, const double *x, double *grad, void *cost_data);