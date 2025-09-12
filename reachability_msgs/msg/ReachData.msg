uint8 NO_FILLED=0
uint8 FILLED=1

uint8 state

geometry_msgs/Pose pose
geometry_msgs/Pose[] reduced_samples

# Orientation 
geometry_msgs/Quaternion[] qs

reachability_msgs/Param[] metrics
reachability_msgs/ReachSample[] samples
