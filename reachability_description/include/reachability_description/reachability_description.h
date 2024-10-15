#ifndef __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__
#define __REACHABILITY_TOOLS_REACHABILITY_DESCRIPTION_H__

#include <rclcpp/rclcpp.hpp>
#include <trac_ik/trac_ik.hpp>

#include <urdf/model.h>
#include <srdfdom/model.h>

#include <robot_unit/fast_robot_collision_object.h>
#include <Eigen/Geometry>
#include <reachability_description_parameters.hpp>

#include <pluginlib/class_loader.hpp>
#include <reachability_description/reach_graph.h>


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
    ReachabilityDescription(const rclcpp::Node::SharedPtr &_node);
    ~ReachabilityDescription();

    bool initialize(const std::string &_robot_name);
    
    bool generateDescription(const std::string &_chain_group);
    bool loadDescription(const std::string &_filename);

    bool storeDescription(const std::string &_chain_group);

    bool viewDescription(const std::string &_chain_group);

    void reach_calc( const double &_min_x, const double &_min_y, const double &_min_z,
                     const double &_max_x, const double &_max_y, const double &_max_z,
                     const reachability_msgs::msg::ChainInfo &_ci,
                     const double &_ik_max_time, const double &_ik_epsilon, 
                     const TRAC_IK::SolveType &_ik_type,
                     const std::map<std::string, KDL::JntArray> &_joint_configs,
                     const std::map<std::string, KDL::Frame> &_fk_poses);


    void estimateReachLimits(const std::string &_chain_group);

    reachability_msgs::msg::ReachData fillData(const std::shared_ptr<ReachGraph> &_reach_graph, 
                                          int _xi, int _yi, int _zi,
                                          const std::shared_ptr<TRAC_IK::TRAC_IK> &_ik_solver,
                                          const reachability_msgs::msg::ChainInfo &_ci,
                                          const std::shared_ptr<robot_unit::RobotCollisionObject> &_rco,
                                          const KDL::JntArray &_q_init);

    bool getReachabilityData(const std::string &_chain_group,
                        const double &_x, const double &_y, const double &_z, 
                         reachability_msgs::msg::ReachData &_data);
    bool getChainInfo(const std::string &_chain_group,
                     reachability_msgs::msg::ChainInfo &_chain_info);
    std::shared_ptr<ReachGraph> getReachGraph(const std::string &_chain_group);
    std::shared_ptr<TRAC_IK::TRAC_IK> getIKSolver(const std::string &_chain_group);
    std::vector<std::pair<double, double>> getJointLimits(const std::string &_chain_group);
    bool isSelfColliding(const sensor_msgs::msg::JointState &_js);
 
    bool addKinematicSolvers(const std::string &_chain_group);
    KDL::JntArray  getClosestJointConfig(const double &_xi, const double &_yi, const double &_zi, 
                                     const std::map<std::string, KDL::JntArray> &_joint_configs, 
                                     const std::map<std::string, KDL::Frame> &_fk_poses);

    protected:

    bool createPluginInstance(std::shared_ptr<ReachGraph> &_rg, 
                              const std::string &_plugin_name);

    bool initializeReachGraph( std::shared_ptr<ReachGraph> &_rg, 
                               const reachability_msgs::msg::ChainInfo &_ci, 
                               double _min_x, double _min_y, double _min_z,
	                       double _max_x, double _max_y, double _max_z,
	                       const double &_resolution, 
                               const uint16_t &_voxel_samples,
	                       const reachability_msgs::msg::ReachData &_reach_default);
	            
    bool initializeReachGraph( std::shared_ptr<ReachGraph> &_rg, 
                               const reachability_msgs::msg::ReachGraph &_msg);	            

    bool writeToDisk(const reachability_msgs::msg::ReachData &_msg,
                     const std::string &_filename);
    bool readFromDisk(const std::string &_filename,
                      reachability_msgs::msg::ReachData &_msg);

    std::string generateDefaultReachGroupName(const std::string &_chain_group);

    void loadParams(const std::string &_chain_group, 
                reachability_description_params::Params &_params);


    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_reach_;

    std::shared_ptr<RobotEntity> re_;
    std::shared_ptr<robot_unit::RobotCollisionObject> rco_;

    std::shared_ptr<pluginlib::ClassLoader<reachability_description::ReachGraph> > reach_graph_loader_;
    std::map<std::string, std::shared_ptr<ReachGraph> > reach_graph_;
    std::map<std::string, std::shared_ptr<TRAC_IK::TRAC_IK> > ik_solver_;
    std::map<std::string, std::shared_ptr<KDL::ChainFkSolverPos_recursive> > fk_solver_;
    std::map<std::string, reachability_msgs::msg::ChainInfo> chain_info_;
    std::map<std::string, std::vector<std::pair<double, double>> > joint_limits_;

    std::string robot_name_;
    std::string urdf_string_;
    std::string srdf_string_;

    std::mutex reach_fill_mutex_;
    std::string plugin_name_;
};

} // namespace reachability_description

#endif //
