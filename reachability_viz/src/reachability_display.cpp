
#include <reachability_viz/reachability_display.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_to_point_cloud2.hpp>
#include <Eigen/Geometry>

namespace reachability_viz
{
using rviz_common::properties::StatusProperty;

ReachabilityDisplay::ReachabilityDisplay() 
: point_cloud_common_(std::make_unique<rviz_default_plugins::PointCloudCommon>(this)),
  marker_common_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this))
{
 nx_ = 0; ny_ = 0; nz_ = 0; plane_dist_ = 0.0;
 top_best_ = 100;
 show_orientation_ = false;

  reach_properties_ = new rviz_common::properties::Property("Reachability", QVariant(),"", this);
  plane_distance_property_ = new rviz_common::properties::FloatProperty("offset", 0.0, "plane offset", reach_properties_, SLOT(updateSlice()), this);
  top_best_metric_property_ = new rviz_common::properties::IntProperty("top_best", 100, "% best", reach_properties_, SLOT(updateSlice()), this);  
  plane_property_ = new rviz_common::properties::EnumProperty("Plane", QString("FULL"), "Plane to slice", reach_properties_, SLOT(updateSlice()), this);

  plane_distance_property_->setMin(-2.0);
  plane_distance_property_->setMax(2.0);
  
  top_best_metric_property_->setMin(0);
  top_best_metric_property_->setMax(100);

  plane_property_->addOptionStd("XY+", 0);
  plane_property_->addOptionStd("XY-", 1);
  plane_property_->addOptionStd("XZ+", 2);
  plane_property_->addOptionStd("XZ-", 3);
  plane_property_->addOptionStd("YZ+", 4);
  plane_property_->addOptionStd("YZ-", 5);
  plane_property_->addOptionStd("FULL", 6);  

  orientation_property_ = new rviz_common::properties::Property("show_orientation", false, "show orientation", reach_properties_, SLOT(updateSlice()), this);

}

void ReachabilityDisplay::onInitialize()
{
  rviz_common::MessageFilterDisplay<reachability_msgs::msg::ReachGraphStamped>::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
  marker_common_->initialize(context_, scene_node_);
                  
  updateSlice();
  
}

void ReachabilityDisplay::updateSlice()
{
  // Get orientation property
  show_orientation_ = orientation_property_->getValue().toBool();

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
   case 6:
     plane = "FULL"; break; 
  }
  double dist = (double)plane_distance_property_->getFloat();

  int top_best = top_best_metric_property_->getInt();
  top_best_ = (double)top_best/100.0;
  
  setPlaneEquationCoefficients(plane, dist, nx_, ny_, nz_, plane_dist_);

  if(last_msg_)
    processMessage(last_msg_);
}



void ReachabilityDisplay::processMessage(const reachability_msgs::msg::ReachGraphStamped::ConstSharedPtr msg)
{
  last_msg_ = msg;
  this->reset();
  sensor_msgs::msg::PointCloud::SharedPtr cloud;
  cloud.reset(new sensor_msgs::msg::PointCloud());
  cloud->header = msg->header;

  visualization_msgs::msg::MarkerArray::SharedPtr marker;
  marker.reset(new visualization_msgs::msg::MarkerArray());

  RCLCPP_INFO(rclcpp::get_logger("reach_display"), "Num points received: %ld! cloud frame: %s", msg->data.points.size(), cloud->header.frame_id.c_str());

  float color;
  uint8_t r, g, b;
  float ratio, green, red;

  sensor_msgs::msg::ChannelFloat32 c;  
  c.name = "rgb";
  cloud->channels.push_back(c);
  
  // Get min and max number of samples
  int min_samples, max_samples;

  getMinMaxSamples(msg, min_samples, max_samples);
  
  int id = 0;
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
    if( nx_*p.x + ny_*p.y + nz_*p.z - plane_dist_ < 0.0)
      continue;

 
    //ratio = (float) pi.samples.size() / (float) msg->data.params.num_voxel_samples;
    ratio = (float)(pi.samples.size() - min_samples) / (float)(max_samples - min_samples);
    
    red = ratio > 0.5? 1.0 - 2.0*(ratio - 0.5) : 1.0;
    green = ratio > 0.5? 1.0 : 2.0*ratio;

    r = (uint8_t)std::floor((int) (red*255.0)); 
    g = (uint8_t)std::floor((int) (green*255.0)); 
    b = 0x00;
    uint32_t col = (r << 16) + (g << 8) + b;
    color = *reinterpret_cast<float*>( &col );
    
    // Only visualize the X% best
    if(ratio > (1.0 - top_best_)) {
    
      cloud->points.push_back(p);
      cloud->channels[0].values.push_back(color);
      
      if(show_orientation_)
      {
         auto ms = generateArrowVizSample(pi, id);
         for(auto ii : ms)
           marker->markers.push_back( ii); // marker->markers.end(), ms.begin(), ms.end()
      }
    } // if ratio > top_best
            
  } // for pi msg.data.points

  
  auto pc2 = rviz_default_plugins::convertPointCloudToPointCloud2(cloud);
  RCLCPP_INFO(rclcpp::get_logger("reach_display"), "Pc2: header: %s, height: %d width: %d data size: %ld", pc2->header.frame_id.c_str(), pc2->height, pc2->width, pc2->data.size());
  
  // Visualize
  point_cloud_common_->addMessage(cloud);    
  marker_common_->addMessage(marker);
}

/**
 * @function getMinMaxSamples
 */
void ReachabilityDisplay::getMinMaxSamples(const reachability_msgs::msg::ReachGraphStamped::ConstSharedPtr &_msg, int &_min_samples, int &_max_samples) {

  _min_samples = 1000;
  _max_samples = 0;

  for(auto pi : _msg->data.points)
  {
     auto num = pi.samples.size();
     if (num < _min_samples)
       _min_samples = num;
     if (num > _max_samples)
       _max_samples = num;
  }
}

/**
 * @function generateArrowVizSample
 */
std::vector<visualization_msgs::msg::Marker> ReachabilityDisplay::generateArrowVizSample(const reachability_msgs::msg::ReachData &_pi, int &_start_id)
{
  std::vector<visualization_msgs::msg::Marker> sample_markers;

  double l = 0.03;
  
  for(auto si : _pi.samples)
  {
    geometry_msgs::msg::Point p1, p2;

    Eigen::Quaterniond q(si.pose.orientation.w, si.pose.orientation.x, si.pose.orientation.y, si.pose.orientation.z);
    Eigen::Matrix3d m; m = q.toRotationMatrix();
         
    p2.x = si.pose.position.x;
    p2.y = si.pose.position.y;
    p2.z = si.pose.position.z;
                          
    p1.x = p2.x - m.col(2).x()*l;
    p1.y = p2.y - m.col(2).y()*l;
    p1.z = p2.z - m.col(2).z()*l;

    visualization_msgs::msg::Marker mi;
    fillDefaultArrowMarker(mi, 0.4, 0.1, 0.4, 1.0);
                                                      
    mi.points.push_back(p1);
    mi.points.push_back(p2);
    mi.id = _start_id;
    
    _start_id++;

    sample_markers.push_back(mi);
  } // end for

  return sample_markers;
}

/**
 * @function fillDefaultArrowMarker
 */
void ReachabilityDisplay::fillDefaultArrowMarker(visualization_msgs::msg::Marker &_mi, const double &_r, const double &_g, const double &_b, const double &_a)
{
  _mi.scale.x = 0.0025; // shaft diameter
  _mi.scale.y = 0.005; // head diameter
  _mi.scale.z = 0.005; // head length (if specified)
  _mi.pose.orientation.w = 1.0;
  _mi.color.r = _r; _mi.color.g = _g; _mi.color.b = _b; _mi.color.a = _a;
  _mi.header.frame_id = last_msg_->header.frame_id;
  
  _mi.header.stamp = rclcpp::Time(0, 0);
  _mi.type = visualization_msgs::msg::Marker::ARROW;

}

void ReachabilityDisplay::update(float wall_dt, float ros_dt)
{
   point_cloud_common_->update(wall_dt, ros_dt);
   marker_common_->update(wall_dt, ros_dt);
}

void ReachabilityDisplay::reset()
{
   rviz_common::MessageFilterDisplay<reachability_msgs::msg::ReachGraphStamped>::reset();
   point_cloud_common_->reset();
   
   marker_common_->clearMarkers();
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
    { _nx = 0; _ny = 0; _nz = 1.0; _d = _plane_dist; }
  else if(_plane == std::string("XY-"))
    { _nx = 0; _ny = 0; _nz = -1.0; _d = _plane_dist; } 
  else if(_plane == std::string("XZ+"))
    { _nx = 0; _ny = 1.0; _nz = 0.0; _d = _plane_dist; }
  else if(_plane == std::string("XZ-"))
    { _nx = 0; _ny = -1.0; _nz = 0.0; _d = _plane_dist; } 
  else if(_plane == std::string("YZ+"))
    { _nx = 1.0; _ny = 0.0; _nz = 0.0; _d = _plane_dist; }
  else if(_plane == std::string("YZ-"))
    { _nx = -1.0; _ny = 0; _nz = 0.0; _d = _plane_dist; } 
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
