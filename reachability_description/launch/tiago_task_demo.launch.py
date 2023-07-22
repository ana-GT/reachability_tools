import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from tiago_description.tiago_launch_utils import (get_tiago_hw_arguments,
						    TiagoXacroConfigSubstitution)

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

    tiago_args = get_tiago_hw_arguments(
        laser_model=True,
        arm=True,
        end_effector=True,
        ft_sensor=True,
        camera_model=True,
        default_end_effector='pal-gripper',
        default_laser_model="sick-571")


    urdf_config = Command(
        [
            ExecutableInPackage(package='xacro', executable="xacro"),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('robots_config'),
                 'robots', 'tiago', 'tiago.urdf.xacro']),
            TiagoXacroConfigSubstitution()
        ])

    parameters = {'robot_description': urdf_config}

    srdf_file = os.path.join(get_package_share_directory('robots_config'), 'config',
                                     'tiago', 'tiago_right-arm_pal-gripper_schunk-ft.srdf')
    srdf_config = open(srdf_file).read()


    robot_description_semantic = {
        'robot_description_semantic': srdf_config
    }

    # Reach parameters
    reachability_yaml = load_yaml(
        "reachability_description", "config/tiago/reachability_params.yaml"
    )
    reachability_params = {"reachability_params": reachability_yaml}

    # markers
    task_marker = Node(
        package='jose',
        executable='robot_to_task_markers_node',
        output='screen',
        parameters=[
            {"group": "arm_torso"},
            {"robot_name": "tiago"}
        ]
    )    

    # Robot to task
    app_robot_to_task = Node(
        package='reachability_description',
        executable='app_robot_to_task',
        output='screen',
        parameters=[
            reachability_params,
            {"robot_description": urdf_config},
            {"robot_description_semantic" : srdf_config},
            {"chain_group_name": "arm_torso"},
            {"robot_name": "tiago"}
        ]
    )    




    return LaunchDescription(
        [*tiago_args,
          task_marker,
          app_robot_to_task
        ]

    )
