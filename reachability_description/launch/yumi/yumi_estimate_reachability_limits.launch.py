import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

#######################################
def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="rviz", default_value="True"),
        DeclareLaunchArgument(name="group", default_value="right_arm"),
    ]

    rc_dir = get_package_share_directory("robots_config")

    # Launch robot
    robot_launch = IncludeLaunchDescription(
            PathJoinSubstitution([rc_dir, 'launch/yumi/yumi_config.launch.py']),
            launch_arguments={
              'rviz': LaunchConfiguration('rviz')
            }.items(),
    )    

    # URDF/SRDF    
    robot_description_config = xacro.process_file(
        os.path.join(rc_dir, "robots/yumi/yumi.urdf.xacro",
        ),
        in_order = False,
        mappings = {'arms_interface': 'VelocityJointInterface', 
                    'grippers_interface': 'EffortJointInterface',
                    'yumi_setup' : 'default'}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}


    robot_description_semantic_config = load_file(
        "robots_config", "config/yumi/yumi.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Reach parameters
    reachability_yaml = load_yaml(
        "reachability_description", "config/yumi/reachability_params.yaml"
    )
    reachability_params = {"reachability_params": reachability_yaml}


    # Reachability limits generation node
    reach_gen = Node(
        package='reachability_description',
        executable='estimate_reachability_limits_node',
        output='screen',
        parameters=[
            reachability_params,
            robot_description,
            robot_description_semantic,
            {"plugin_name": "reachability_description::ReachGraphReuleaux"},            
            {"chain_group_name": LaunchConfiguration("group")}, # right_arm
            {"robot_name": "yumi"} 
        ]
    ) 

    return LaunchDescription(
        launch_args + 
        [robot_launch, reach_gen]
    )
