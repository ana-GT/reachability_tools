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
            PathJoinSubstitution([rc_dir, 'launch/panda_husky/panda_husky_config.launch.py']),
            launch_arguments={
              'rviz': LaunchConfiguration('rviz')
            }.items(),
    ) 

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("robots_config"),
            "robots", "panda_husky",
            "panda_husky.urdf.xacro",
        ),
        mappings ={'hand': 'true'}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    srdf_file = os.path.join(get_package_share_directory('robots_config'),'config',
                                              'panda_husky',
                                              'panda_husky.srdf.xacro')
    srdf_config = Command(
        [FindExecutable(name='xacro'), ' ', srdf_file, ' hand:=true']
    )
    robot_description_semantic = {'robot_description_semantic': srdf_config}

    # Reach parameters
    reachability_yaml = load_yaml(
        "reachability_description", "config/panda_husky/reachability_params.yaml"
    )
    reachability_params = {"reachability_params": reachability_yaml}


    # Panda_husky
    reach_gen = Node(
        package='reachability_description',
        executable='generate_reachability_node',
        output='screen',
        parameters=[
            reachability_params,
            robot_description,
            robot_description_semantic,
            {"chain_group_name": "arm_base"}, # arm
            {"robot_name": "panda_husky"},
            {"plugin_name": "reachability_description::ReachGraphReuleaux"} 
        ]
    )

    return LaunchDescription(
        launch_args +
        [robot_launch, reach_gen]
    )
