import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

#####################################
# Helpers functions
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

###########################################
def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="rviz", default_value="True"),
    ]

    rc_dir = get_package_share_directory("robots_config")

    # Launch robot
    robot_launch = IncludeLaunchDescription(
            PathJoinSubstitution([rc_dir, 'launch/fetch/fetch_config.launch.py']),
            launch_arguments={
              'rviz': LaunchConfiguration('rviz')
            }.items(),
    )    

    # URDF/SRDF
    robot_description_config = xacro.process_file( os.path.join(rc_dir, "robots/fetch/fetch.urdf.xacro") )
    robot_description = {"robot_description": robot_description_config.toxml()}

    srdf_file = os.path.join(rc_dir, 'config/fetch/fetch.srdf')
    srdf_config = open(srdf_file).read()

    robot_description_semantic = {'robot_description_semantic': srdf_config}

    # Reach parameters
    reachability_yaml = load_yaml("reachability_description", "config/fetch/reachability_params.yaml")
    reachability_params = {"reachability_params": reachability_yaml}
    

    # Reachability limits generation node
    reach_gen = Node(
        package='reachability_description',
        executable='estimate_reachability_limits_node',
        output='screen',
        parameters=[
            # Reachability description parameter
            reachability_params,
            robot_description,
            robot_description_semantic,
            {"plugin_name": "reachability_description::ReachGraphReuleaux"},
            {"chain_group_name": "arm_with_torso"}, # arm
            {"robot_name": "fetch"}
        ]
    )    


    return LaunchDescription(
        launch_args + 
        [robot_launch, reach_gen]
    )
