
#include <reachability_viz/reachability_display.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_to_point_cloud2.hpp>
namespace reachability_viz
{
using rviz_common::properties::StatusProperty;

ReachabilityDisplay::ReachabilityDisplay() 
: point_cloud_common_(std::make_unique<rviz_default_plugins::PointCloudCommon>(this))
{

}

void ReachabilityDisplay::onInitialize()
{
  rviz_common::MessageFilterDisplay<reachability_msgs::msg::ReachGraphStamped>::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

void ReachabilityDisplay::processMessage(const reachability_msgs::msg::ReachGraphStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("reach_display"), "process message!!!!");
  sensor_msgs::msg::PointCloud::SharedPtr cloud;
  cloud.reset(new sensor_msgs::msg::PointCloud());
  cloud->header = msg->header;

  RCLCPP_INFO(rclcpp::get_logger("reach_display"), "Num points received: %d!!!!!!!!!!!!!!!!!!!!!!!!! cloud frame: %s", msg->data.points.size(), cloud->header.frame_id.c_str());

  sensor_msgs::msg::ChannelFloat32 c;  
  c.name = "rgb";
  
  cloud->channels.push_back(c);
  for(auto pi : msg->data.points)
  {
    geometry_msgs::msg::Point32 p;

    
    if(pi.samples.empty())
      continue;
      
    auto position = pi.samples[0].pose.position;
    p.x = position.x;
    p.y = position.y;
    p.z = position.z;
  
    uint32_t color;
    uint8_t r, g, b;
    r = 0xFF; g = 0xFF; b = 0x00;
    color = (r << 16) + (g << 8) + b;
    
    cloud->points.push_back(p);
    cloud->channels[0].values.push_back(color);
    
  }   RCLCPP_INFO(rclcpp::get_logger("reach_display"), "Cloud size: %d", cloud->points.size());

  
  auto pc2 = rviz_default_plugins::convertPointCloudToPointCloud2(cloud);
  RCLCPP_INFO(rclcpp::get_logger("reach_display"), "Pc2: header: %s, height: %d width: %d data size: %d", pc2->header.frame_id.c_str(), pc2->height, pc2->width, pc2->data.size());
  point_cloud_common_->addMessage(cloud);  
}

void ReachabilityDisplay::update(float wall_dt, float ros_dt)
{
   point_cloud_common_->update(wall_dt, ros_dt);
}

void ReachabilityDisplay::reset()
{
   rviz_common::MessageFilterDisplay<reachability_msgs::msg::ReachGraphStamped>::reset();
   point_cloud_common_->reset();
}

void ReachabilityDisplay::onDisable()
{
   rviz_common::MessageFilterDisplay<reachability_msgs::msg::ReachGraphStamped>::onDisable();
   point_cloud_common_->onDisable();
}


}  // namespace reachability_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(reachability_viz::ReachabilityDisplay, rviz_common::Display)
