import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
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


#####################################
def generate_launch_description():

    mars_rover_models_path = get_package_share_directory('robots_config')
    urdf_model_path = os.path.join(mars_rover_models_path, 'robots', 'curiosity',
        'curiosity_base_link.urdf.xacro')

    doc = xacro.process_file(urdf_model_path)
    robot_description = {'robot_description': doc.toxml()}
        
    srdf_file = os.path.join(get_package_share_directory('robots_config'), 'config',
                                     'curiosity', 'curiosity.srdf')
    srdf_config = open(srdf_file).read()

    robot_description_semantic = {
        'robot_description_semantic': srdf_config
    }

    # Robot task UI Params
    rtu_yaml = load_yaml(
        "task_ui", "config/curiosity_robot_task.yaml"
    )
    rtu_params = {"robot_task_ui_params": rtu_yaml}

    # Robot task UI node
    task_marker = Node(
        package='task_ui',
        executable='markers_get_robot_base_node',
        output='screen',
        parameters=[
            {"group": "arm"},
            {"robot_name": "curiosity"},
            rtu_params
        ]
    )    

    # Reach parameters
    reachability_yaml = load_yaml(
        "reachability_description", "config/curiosity/reachability_params.yaml"
    )
    reachability_params = {"reachability_params": reachability_yaml}


    # Robot to task
    app_robot_to_task = Node(
        package='reachability_description',
        executable='app_robot_to_task_2',
        output='screen',
        parameters=[
            reachability_params,
            robot_description,
            robot_description_semantic,
            {"chain_group_name": "arm"},
            {"robot_name": "curiosity"}
        ]
    )    


    return LaunchDescription(
        [
            task_marker,
            app_robot_to_task
        ]

    )
