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

from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch_param_builder import load_xacro
from pathlib import Path

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


#####################################
def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument("plugin_name", default_value="rach_ik_plugins::MobileHeuristicOptimizer"),
        DeclareLaunchArgument("group_name", default_value="arm_with_torso")
    ]

    config_pkg = get_package_share_directory('robots_config')

    robot_launch = IncludeLaunchDescription(
            PathJoinSubstitution([config_pkg, 'launch', 'fetch/fetch_config.launch.py'])
    )


    # markers
    rtu_yaml = load_yaml("task_ui", "config/fetch_robot_task.yaml")
    rtu_params = {"robot_task_ui_params": rtu_yaml}
    
    task_marker = Node(
        package='task_ui',
        executable='markers_get_reach_data_node',
        output='screen',
        parameters=[
            {"group": LaunchConfiguration("group_name")},
            {"robot_name": "fetch"},
            rtu_params
        ],
    )    

    # Robot to task
    rc_dir = get_package_share_directory("robots_config")
    robot_description_config = xacro.process_file(os.path.join(rc_dir, "robots/fetch/fetch.urdf.xacro"))
    urdf_config = robot_description_config.toxml()

    srdf_file = os.path.join(rc_dir, 'config/fetch/fetch.srdf')
    srdf_config = open(srdf_file).read()

    reachability_yaml = load_yaml("reachability_description", "config/fetch/reachability_params.yaml")
    
    reach_data = Node(
        package='reachability_applications',
        executable='simple_get_reach_data_node',
        output='screen',
        parameters=[
            # Reachability Description parameters
            {"reachability_params": reachability_yaml},           
            {"robot_description": urdf_config},
            {"robot_description_semantic" : srdf_config},
            {"plugin_name": "reachability_description::ReachGraphReuleaux"},
            {"chain_group_name": LaunchConfiguration("group_name")},
            {"robot_name": "fetch"}
        ],

    )

    return LaunchDescription(
        launch_args + 
        [
          robot_launch,
          task_marker,
          reach_data
        ]

    )
