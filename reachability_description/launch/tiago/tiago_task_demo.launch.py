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

    xacro_file_path = Path(
        os.path.join(
            get_package_share_directory("robots_config"),
            "robots", 'tiago',
            "tiago.urdf.xacro",
        )
    )

    xacro_input_args = {
        "arm_type": "tiago-arm",
        "camera_model": "orbbec-astra",
        "end_effector": "pal-gripper",
        "ft_sensor": "schunk-ft",
        "laser_model": "sick-571",
        "wrist_model": "wrist-2010",
        "base_type": "pmb2",
        "has_screen": False,
#        "use_sim_time": False,
#        "is_public_sim": True,
#        "namespace": read_launch_argument("namespace", context),
    }
    urdf_config = load_xacro(xacro_file_path, xacro_input_args)
    
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

    # Robot task UI Params
    rtu_yaml = load_yaml(
        "task_ui", "config/tiago_robot_task.yaml"
    )
    rtu_params = {"robot_task_ui_params": rtu_yaml}

    # markers
    task_marker = Node(
        package='task_ui',
        executable='robot_task_markers_node',
        output='screen',
        parameters=[
            {"group": "arm_torso"},
            {"robot_name": "tiago"},
            rtu_params
        ]
    )    

    # Robot to task
    app_robot_to_task = Node(
        package='reachability_description',
        executable='app_robot_to_task_2',
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
        [
          task_marker,
          app_robot_to_task
        ]

    )
