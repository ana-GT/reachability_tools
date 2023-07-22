#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <reachability_msgs/srv/set_robot_pose.hpp>

/**
 * @class MoveBase
 */
class MoveBase
{
public:
  MoveBase(rclcpp::Node::SharedPtr _node ) :
  nh_(_node)
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh_);

  }

  bool initialize( const std::string &_ref_frame, const std::string &_robot_frame)
  {
    // Start with a zero pose
    ref_frame_ = _ref_frame;
    robot_frame_ = _robot_frame;

    geometry_msgs::msg::Pose zero_pose;
    zero_pose.orientation.w = 1.0;
    updatePose(zero_pose, ref_frame_);

    // Set service server
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = nh_->create_service<reachability_msgs::srv::SetRobotPose>("set_robot_pose", 
                std::bind(&MoveBase::handleSrv, this, _1, _2));

    return true;
  }

void handleSrv(const std::shared_ptr<reachability_msgs::srv::SetRobotPose::Request> req,
               std::shared_ptr<reachability_msgs::srv::SetRobotPose::Response> res)
  {
    // Update
    RCLCPP_INFO(nh_->get_logger(), "Got service call to move base to %f %f %f", req->pose.pose.position.x,
                req->pose.pose.position.y, req->pose.pose.position.z); 
    updatePose(req->pose.pose, req->pose.header.frame_id);
    res->success = true;
  }

void updatePose(geometry_msgs::msg::Pose _pose, std::string _frame_id)
{
    tfx_.transform.translation.x = _pose.position.x;
    tfx_.transform.translation.y = _pose.position.y;
    tfx_.transform.translation.z = _pose.position.z;
    tfx_.transform.rotation = _pose.orientation;
    tfx_.header.stamp = nh_->get_clock()->now();

    tfx_.header.frame_id = _frame_id;
    if(_frame_id.empty())
        tfx_.header.frame_id = ref_frame_;
    tfx_.child_frame_id = robot_frame_;
}

void publishPose()
{
    tfx_.header.stamp = nh_->get_clock()->now();
    tf_static_broadcaster_->sendTransform(tfx_);
}

protected:
   rclcpp::Node::SharedPtr nh_;
   rclcpp::Service<reachability_msgs::srv::SetRobotPose>::SharedPtr srv_;
   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
   std::string ref_frame_;
   std::string robot_frame_;

   geometry_msgs::msg::TransformStamped tfx_;

  };

/////////////////////////////////////////
int main(int argc, char* argv[])
{
 rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_base");
    RCLCPP_INFO(rclcpp::get_logger("move_base"), "MoveBase");

  // Read parameters
  std::string ref_frame;
  std::string robot_frame;
  if(!node->has_parameter("ref_frame"))
    node->declare_parameter("ref_frame", std::string(""));
  node->get_parameter("ref_frame", ref_frame);

  if(!node->has_parameter("robot_frame"))
    node->declare_parameter("robot_frame", std::string(""));
  node->get_parameter("robot_frame", robot_frame);

  // Hand to user app
  MoveBase mb(node);
  if(!mb.initialize(ref_frame, robot_frame))
    return 1;

  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
    mb.publishPose();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;    

}