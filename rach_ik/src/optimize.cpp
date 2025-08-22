#include <rach_ik/optimize.h>
#include <rach_ik/constraint.h>
#include <rach_ik/objective.h>
#include <math.h>
#include <stdio.h>

#include <vector>
#include <iostream>
#include <iomanip>
#include <nlopt.hpp>


RachOptimizer::RachOptimizer() :
rclcpp::Node("rach_optimizer") {
    this->declare_parameter("urdf_string", std::string(""));
    this->declare_parameter("srdf_string", std::string(""));    

}

/**
 * @function init
 */
bool RachOptimizer::init() {

    this->get_parameter("urdf_string", urdf_string_);
    this->get_parameter("srdf_string", srdf_string_);

    if(urdf_string_.empty())
        return false;

    if(srdf_string_.empty())
        return false;

    // Initialize the robot entity
    if(!cd_.re.init(urdf_string_, srdf_string_))
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't load robot entity");
        return false;
    }    

    setUserInterface();

    return true;
}

void RachOptimizer::setUserInterfaces() {

    // Offer a service 
    srv_ik_ = this->create_service<reachability_msgs::srv::GetIKPose>("get_ik_pose", 
                        std::bind(&RachOptimizer::handleIKRequest, this, _1, _2));
}

void RachOptimizer::handleIKRequest(const std::shared_ptr<reachability_msgs::srv::GetIKPose::Request> req,
                                    std::shared_ptr<reachability_msgs::srv::GetIKPose::Response> res)
{

}

bool RachOptimizer::getConfiguration() {

nlopt::opt opt(nlopt::LD_MMA, 2);

std::vector<double> lb(2);
lb[0] = -HUGE_VAL; lb[1] = 0.0;
opt.set_lower_bounds(lb);

opt.set_min_objective(cost_function, NULL);

constraint_data data[2] = { {2, 0}, {-1, 1}};
opt.add_inequality_constraint(constraint_lin, &data[0], 1e-8);
opt.add_inequality_constraint(constraint_lin, &data[1], 1e-8);

opt.set_xtol_rel(1e-4);


std::vector<double> x = {1.235, 5.678};
double minf;
bool ret;

try {
    nlopt::result result = opt.optimize(x, minf);
    RCLCPP_INFO(this->get_logger(), "Found minimum at f(%f, %f) = %f", x[0], x[1], minf);
    ret = true;
} catch(std::exception &e) {
    RCLCPP_INFO(this->get_logger(), "nlopt failed: %s", e.what() );
    ret = false;
}

return ret;
}
