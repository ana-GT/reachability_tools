
#pragma once

#include <memory>

#include <reachability_msgs/msg/reach_graph_stamped.hpp>

#include <rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp>
#include <rviz_default_plugins/visibility_control.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/message_filter_display.hpp>

namespace reachability_viz
{
class RVIZ_DEFAULT_PLUGINS_PUBLIC ReachabilityDisplay
  : public rviz_common::MessageFilterDisplay<reachability_msgs::msg::ReachGraphStamped>
{
 Q_OBJECT
public:
  ReachabilityDisplay();
  void reset() override;
  void update(float wall_dt, float ros_dt) override;
  void onDisable() override;
    
protected:
  void onInitialize() override;
  void processMessage(const reachability_msgs::msg::ReachGraphStamped::ConstSharedPtr msg) override;
  bool setPlaneEquationCoefficients(const std::string &_plane, 
                                    const double &_plane_dist,
                                    double &_nx, double &_ny, double &_nz, double &_d);
  std::unique_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common_;
  std::unique_ptr<rviz_common::properties::EnumProperty> plane_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> plane_distance_property_;  
  
  reachability_msgs::msg::ReachGraphStamped::ConstSharedPtr last_msg_;
  double nx_, ny_, nz_, plane_dist_;
  
private Q_SLOTS:
  void updateSlice();

};

}  // namespace reachability_viz


