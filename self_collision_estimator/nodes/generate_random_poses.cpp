/**
 * @file generate_random_poses.cpp
 */
#include <rclcpp/rclcpp.hpp>
#include <robot_unit/fast_robot_collision_object.h>
#include <robot_unit/robot_entity.h>

/** 
 * @class GenerateRandomPoses
 */
class GenerateRandomPoses : public rclcpp::Node {

 public:
   GenerateRandomPoses() :
   Node("generate_random_poses")
   {
     this->declare_parameter<int>("num_poses");
     this->declare_parameter<std::string>("urdf_string");
     this->declare_parameter<std::string>("srdf_string");
     this->declare_parameter<std::string>("group_name");          
     this->declare_parameter<std::string>("robot_name");     
   }
   
   bool init() {
     this->get_parameter("num_poses", num_poses_);
     this->get_parameter("urdf_string", urdf_string_);
     this->get_parameter("srdf_string", srdf_string_);
     this->get_parameter("robot_name", robot_name_);
     this->get_parameter("group_name", group_name_);     
          
     reference_frame_ = "world";
     
     if(!re_.init(urdf_string_, srdf_string_))
       return false;
     
     if(!rco_.init(reference_frame_, robot_name_, urdf_string_, srdf_string_))
       return false;
       
     // Get joint names
     reachability_msgs::msg::ChainInfo chain_info_;
     if(!re_.getChainInfo(group_name_, chain_info_))
       return false;
       
     joint_names_ = chain_info_.joint_names;               
                    
     // Get joint limits  
     re_.getJointLimits(joint_names_, joint_limits_);  
     return true;
   }


   double generateRand(double _min, double _max)
   {
     double val = (_max - _min) * ( (double)rand() / (double)RAND_MAX ) + _min;
     return val;
   }

   sensor_msgs::msg::JointState generateRandomPose() {
     sensor_msgs::msg::JointState js;
     
     js.name = joint_names_;
     js.position.resize(joint_names_.size()); 
     for(int i = 0; i < joint_names_.size(); ++i)
       js.position[i] = generateRand(joint_limits_[i].first, joint_limits_[i].second);
     
     return js;
   }
   
   bool generateRandomPoses() {
   
     int num_self_collide_confs = 0;
     int num_collision_free_confs = 0;
     
     for(int i = 0; i < num_poses_; ++i)
     {      
        auto js = generateRandomPose();
        rco_.update(js);
        if(rco_.selfCollide())
          num_self_collide_confs++;
        else 
          num_collision_free_confs++;
     }
     
     RCLCPP_INFO(this->get_logger(), "Self-collision samples: %d. Collision-free: %d. Percentage self-collision: %f", num_self_collide_confs, num_collision_free_confs, (double) num_self_collide_confs / (double) num_collision_free_confs*100.0 );
     return true;
   }

  protected:
  
  int num_poses_;
  std::string urdf_string_;
  std::string srdf_string_; 
  robot_unit::RobotCollisionObject rco_;
  RobotEntity re_;
  std::string reference_frame_;
  std::string robot_name_;
  std::string group_name_;
  
  std::vector<std::pair<double, double>> joint_limits_;

  std::vector<std::string> joint_names_;

};

//////////////////////////////////////
int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   auto grp = std::make_shared<GenerateRandomPoses>();

   if(!grp->init())
   {
      RCLCPP_ERROR(grp->get_logger(), "Error initializing object");
      return 0;
   }

   RCLCPP_WARN(grp->get_logger(), "Generate random poses");
   grp->generateRandomPoses();
  
   rclcpp::spin(grp);
   return 0;
}
