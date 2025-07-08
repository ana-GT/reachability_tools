
Pre-requisites
===============
This package is tested in ROS2 Jazzy, Ubuntu 24.04

Build workspace
================

1. Clone this workspace repository that have scripts to pull all the pre-requisites and packages needed to run the demos:
   ```
   cd ~
   git clone git@github.com:ana-GT/compose_setups.git reachability_ws
   cd reachability_ws
   ```
2. Clone the necessary packages (including this one), and ignore some packages that are not needed:
   ```
   ./scripts/clone_rosws.sh
   ./scripts/ignore.sh
   ```
3. Build:
   ```
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   ```
   
Visualize stored reachability
==============================

```
ros2 launch reachability_description fetch_load_reachability.launch.py
```   
   
Test Reachability
==================

1. Start Fetch demo:
   ```
   ros2 launch reachability_demos fetch_reachability_query.launch.py
   ```
   
   Now you can move the gimbal around, right-click and press Get Pose to see the possible poses
