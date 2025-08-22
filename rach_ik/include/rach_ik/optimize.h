#pragma once

#include <rclcpp/rclcpp.hpp>

/**
 * @class RachOptimizer
 */
class RachOptimizer : public rclcpp::Node {

public :
    RachOptimizer();
    bool init();
    bool getConfiguration();

protected:
    std::string urdf_string_;

};