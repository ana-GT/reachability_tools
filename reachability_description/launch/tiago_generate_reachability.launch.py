from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def load_file(file_path):
    try:
        with open(file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


parameters = [
  {'name': 'robot_description_file',                'description': 'Path to the URDF/xacro file',                     'default': PathJoinSubstitution([FindPackageShare('robots_config'), 'config', 'tiago', 'reach_study.xacro'])},
  {'name': 'robot_description_semantic_file',       'description': 'Path to the SRDF file',                           'default': PathJoinSubstitution([FindPackageShare('robots_config'), 'config', 'tiago', 'tiago_pal-hey5.srdf'])}
#  {'name': 'config_file',                           'description': 'Path to the reach study configuration YAML file', 'default': PathJoinSubstitution([FindPackageShare('reach_ros'), 'demo', 'config', 'reach_study.yaml'])},
#  {'name': 'config_name',                           'description': 'Reach study configuration name',                  'default': 'reach_study'},
#  {'name': 'results_dir',                           'description': 'Directory in which to save reach study results',  'default': PathJoinSubstitution([FindPackageShare('reach_ros'), 'demo', 'results'])},
]


def declare_launch_arguments():
    return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in parameters]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments() + [OpaqueFunction(function=launch)])


def launch(context, *args, **kwargs):
    robot_description_file = LaunchConfiguration('robot_description_file')
    robot_description_semantic_file = LaunchConfiguration('robot_description_semantic_file')

#    robot_description = ParameterValue(Command(['xacro ', robot_description_file]), value_type=str)


    robot_description_semantic = load_file(robot_description_semantic_file.perform(context))

    os.environ['REACH_PLUGINS'] = 'reach_ros_plugins'

    return [
        Node(
            package='reachability_description',
            executable='generate_reachability_node',
            output='screen',
            parameters=[{
                'robot_description': generate_robot_description_action(),
                'robot_description_semantic': robot_description_semantic
            }]
        )
    ]