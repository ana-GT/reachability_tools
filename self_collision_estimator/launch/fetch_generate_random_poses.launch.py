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

    config_pkg = get_package_share_directory('robots_config')

    robot_launch = IncludeLaunchDescription(
            PathJoinSubstitution([config_pkg, 'launch', 'fetch/fetch_config.launch.py'])
    )


    rc_dir = get_package_share_directory("robots_config")
    robot_description_config = xacro.process_file(os.path.join(rc_dir, "robots/fetch/fetch.urdf.xacro"))
    urdf_config = robot_description_config.toxml()

    srdf_file = os.path.join(rc_dir, 'config/fetch/fetch.srdf')
    srdf_config = open(srdf_file).read()

    random_poses = Node(
        package='self_collision_estimator',
        executable='generate_random_poses',
        output='screen',
        parameters=[
            {"urdf_string": urdf_config},
            {"srdf_string" : srdf_config},
            {"num_poses": 50000},
            {"group_name": "arm_with_torso"},
            {"robot_name": "fetch"}
        ],
        #prefix=["xterm -e gdb -ex run --args"]

    )

    return LaunchDescription(
        [
          #robot_launch,
          random_poses
        ]

    )
