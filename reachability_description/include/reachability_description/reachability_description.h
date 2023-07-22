#ifndef __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__
#define __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__

#include <rclcpp/rclcpp.hpp>
#include <trac_ik/trac_ik.hpp>

#include <urdf/model.h>
#include <srdfdom/model.h>

#include <robot_unit/fast_robot_collision_object.h>
#include <Eigen/Geometry>
#include <reachability_description/reach_data.h>

#include <reachability_description_parameters.hpp>

namespace reachability_description
{

#define DEFAULT_REF_FRAME "world"
#define REACH_CLOUD_TOPIC "reach_data_test"

/**
 * @class ReachabilityDescription
 */
class ReachabilityDescription
{
    public:
    ReachabilityDescription(const rclcpp::Node::SharedPtr &_nh);
    ~ReachabilityDescription();

    bool initialize(const std::string &_robot_name);
    bool quickTest(const std::string &_chain_group);
    
    bool generateDescription(const std::string &_chain_group);
    bool loadDescription(const std::string &_filename);

    bool storeDescription(const std::string &_chain_group);

    bool viewDescription(const std::string &_chain_group);

    void reach_calc( const double &_min_x, const double &_min_y, const double &_min_z,
                     const double &_max_x, const double &_max_y, const double &_max_z,
                     const reachability_msgs::msg::ChainInfo &_ci,
                     const double &_ik_max_time, const double &_ik_epsilon, const TRAC_IK::SolveType &_ik_type);


    void estimateReachLimits(const std::string &_chain_group);

    reachability_msgs::msg::ReachData fillData(const std::shared_ptr<ReachGraph> &_reach_graph, 
                                          int _xi, int _yi, int _zi,
                                          const std::shared_ptr<TRAC_IK::TRAC_IK> &_ik_solver,
                                          const reachability_msgs::msg::ChainInfo &_ci,
                                          const std::shared_ptr<robot_unit::RobotCollisionObject> &_rco);

    bool getReachabilityData(const double &_x, const double &_y, const double &_z, 
                         reachability_msgs::msg::ReachData &_data);
    
    std::shared_ptr<ReachGraph> getReachGraph();

    protected:

    bool writeToDisk(const reachability_msgs::msg::ReachData &_msg,
                     const std::string &_filename);
    bool readFromDisk(const std::string &_filename,
                      reachability_msgs::msg::ReachData &_msg);

    std::string generateDefaultReachGroupName(const std::string &_chain_group);

    void loadParams(const std::string &_chain_group, 
                std::shared_ptr<reachability_description_params::ParamListener> _param_listener,
                reachability_description_params::Params _params);

    rclcpp::Node::SharedPtr nh_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_reach_;

    std::shared_ptr<RobotEntity> re_;
    std::shared_ptr<robot_unit::RobotCollisionObject> rco_;

    std::shared_ptr<ReachGraph> reach_graph_;
    std::string robot_name_;
    std::string urdf_string_;
    std::string srdf_string_;

    std::mutex reach_fill_mutex_;

};

} // namespace reachability_description

#endif //