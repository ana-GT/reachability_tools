
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
 nx_ = 0; ny_ = 0; nz_ = 0; plane_dist_ = 0.0;
}

void ReachabilityDisplay::onInitialize()
{
  rviz_common::MessageFilterDisplay<reachability_msgs::msg::ReachGraphStamped>::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
  
  plane_property_ = std::make_unique<rviz_common::properties::EnumProperty>(
    "Plane", QString("XY+"), "Plane to slice", this, SLOT(updateSlice()));
    
  plane_property_->addOptionStd("XY+", 0);
  plane_property_->addOptionStd("XY-", 1);
  plane_property_->addOptionStd("XZ+", 2);
  plane_property_->addOptionStd("XZ-", 3);
  plane_property_->addOptionStd("YZ+", 4);
  plane_property_->addOptionStd("YZ-", 5);

  plane_distance_property_ = std::make_unique<rviz_common::properties::FloatProperty>("offset", 0.0, "plane offset", this, SLOT(updateSlice()));
            
  updateSlice();
  
}

void ReachabilityDisplay::updateSlice()
{
  // Get plane and distance
  int plane_int = plane_property_->getOptionInt();
  std::string plane;
  switch(plane_int)
  {
   case 0:
     plane = "XY+"; break;
   case 1:
     plane = "XY-"; break;
   case 2:
     plane = "XZ+"; break;
   case 3:
     plane = "XZ-"; break;
   case 4:
     plane = "YZ+"; break;
   case 5:
     plane = "YZ-"; break;          
  }
  double dist = (double)plane_distance_property_->getFloat();
  
  setPlaneEquationCoefficients(plane, dist, nx_, ny_, nz_, plane_dist_);
  
  if(last_msg_)
    processMessage(last_msg_);
}



void ReachabilityDisplay::processMessage(const reachability_msgs::msg::ReachGraphStamped::ConstSharedPtr msg)
{
  last_msg_ = msg;
  
  RCLCPP_INFO(rclcpp::get_logger("reach_display"), "process message!");
  sensor_msgs::msg::PointCloud::SharedPtr cloud;
  cloud.reset(new sensor_msgs::msg::PointCloud());
  cloud->header = msg->header;

  RCLCPP_INFO(rclcpp::get_logger("reach_display"), "Num points received: %d! cloud frame: %s", msg->data.points.size(), cloud->header.frame_id.c_str());

  float color;
  uint8_t r, g, b, a;
  float ratio, green, red;

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
 
    // If not above plane, don't show point
    if( nx_*p.x + ny_*p.y + nz_*p.z + plane_dist_ < 0.0)
      continue;

 
    ratio = (float) pi.samples.size() / (float) msg->data.params.num_voxel_samples;
    red = ratio > 0.5? 1.0 - 2.0*(ratio - 0.5) : 1.0;
    green = ratio > 0.5? 1.0 : 2.0*ratio;

    r = (uint8_t)std::floor((int) (red*255.0)); 
    g = (uint8_t)std::floor((int) (green*255.0)); 
    b = 0x00;
    uint32_t col = (r << 16) + (g << 8) + b;
    color = *reinterpret_cast<float*>( &col );
    
    cloud->points.push_back(p);
    cloud->channels[0].values.push_back(color);
        
  }

  RCLCPP_INFO(rclcpp::get_logger("reach_display"), "Cloud size: %d", cloud->points.size());
 
  
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

/**
 * @function setPlanEquationCoefficients
 */
bool ReachabilityDisplay::setPlaneEquationCoefficients(const std::string &_plane, 
                                              const double &_plane_dist,
                                              double &_nx, double &_ny, double &_nz, double &_d)
{
  if(_plane == std::string("XY+"))
    { _nx = 0; _ny = 0; _nz = -1.0; _d = _plane_dist; }
  else if(_plane == std::string("XY-"))
    { _nx = 0; _ny = 0; _nz = 1.0; _d = -_plane_dist; } 
  else if(_plane == std::string("XZ+"))
    { _nx = 0; _ny = -1.0; _nz = 0.0; _d = _plane_dist; }
  else if(_plane == std::string("XZ-"))
    { _nx = 0; _ny = 1.0; _nz = 0.0; _d = -_plane_dist; } 
  else if(_plane == std::string("YZ+"))
    { _nx = -1.0; _ny = 0.0; _nz = 0.0; _d = _plane_dist; }
  else if(_plane == std::string("YZ-"))
    { _nx = 1.0; _ny = 0; _nz = 0.0; _d = -_plane_dist; } 
  else if(_plane == std::string("FULL"))
  { _nx = 0; _ny = 0; _nz = 0; _d = 0; }
  else
  {
     RCLCPP_ERROR(rclcpp::get_logger("ReachGraph"), " plane parameter is not set up with a valid string!");
     return false;
  }
  
  return true;
}


}  // namespace reachability_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(reachability_viz::ReachabilityDisplay, rviz_common::Display)
