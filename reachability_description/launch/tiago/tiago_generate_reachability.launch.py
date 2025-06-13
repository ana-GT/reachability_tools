import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
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
        DeclareLaunchArgument(name="rviz", default_value="True"),
    ]

    rc_dir = get_package_share_directory("robots_config")

    # Launch robot
    robot_launch = IncludeLaunchDescription(
            PathJoinSubstitution([rc_dir, 'launch/tiago/tiago_config.launch.py']),
            launch_arguments={
              'rviz': LaunchConfiguration('rviz')
            }.items(),
    )    

    # URDF/SRDF
    robot_config = get_package_share_directory("robots_config")
    xacro_file_path = Path(os.path.join(robot_config, "robots/tiago/tiago.urdf.xacro"))

    xacro_input_args = {
        "arm_type": "tiago-arm",
        "camera_model": "orbbec-astra",
        "end_effector": "pal-gripper",
        "ft_sensor": "schunk-ft",
        "laser_model": "sick-571",
        "wrist_model": "wrist-2010",
        "base_type": "pmb2",
        "has_screen": False
    }
    urdf_config = load_xacro(xacro_file_path, xacro_input_args)    
    robot_description = {'robot_description': urdf_config}

    srdf_file = os.path.join(rc_dir, 'config/tiago/tiago_right-arm_pal-gripper_schunk-ft.srdf')
    srdf_config = open(srdf_file).read()
    robot_description_semantic = {'robot_description_semantic': srdf_config}


    # Reach parameters
    reachability_yaml = load_yaml(
        "reachability_description", "config/tiago/reachability_params.yaml"
    )
    reachability_params = {"reachability_params": reachability_yaml}


    # Reachability generation node
    reach_gen = Node(
        package='reachability_description',
        executable='generate_reachability_node',
        output='screen',
        parameters=[
            reachability_params,
            robot_description,
            robot_description_semantic,
            {"plugin_name": "reachability_description::ReachGraphReuleaux"},
            {"chain_group_name": "arm_torso"}, # arm_torso, arm
            {"robot_name": "tiago"},
        ] #, prefix=['xterm -e gdb -ex run --args']
    )    


    return LaunchDescription(
        launch_args +
        [robot_launch, reach_gen]
    )
