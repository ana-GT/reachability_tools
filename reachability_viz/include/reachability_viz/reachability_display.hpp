
#pragma once

#include <memory>

#include <reachability_msgs/msg/reach_graph_stamped.hpp>

#include <rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp>
#include <rviz_default_plugins/visibility_control.hpp>
#include <rviz_common/message_filter_display.hpp>

namespace reachability_viz
{
class RVIZ_DEFAULT_PLUGINS_PUBLIC ReachabilityDisplay
  : public rviz_common::MessageFilterDisplay<reachability_msgs::msg::ReachGraphStamped>
{

public:
  ReachabilityDisplay();
  void reset() override;
  void update(float wall_dt, float ros_dt) override;
  void onDisable() override;
  
protected:
  void onInitialize() override;
  void processMessage(const reachability_msgs::msg::ReachGraphStamped::ConstSharedPtr msg) override;

  std::unique_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common_;
};

}  // namespace reachability_viz


