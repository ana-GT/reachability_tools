
#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

void jointStateToVector(const sensor_msgs::msg::JointState &_js,
                        std::vector<double> &_x);

