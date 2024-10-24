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

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("robots_config"),
            "robots", "panda",
            "panda.urdf.xacro",
        ),
        mappings ={'hand': 'true'}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    srdf_file = os.path.join(get_package_share_directory('robots_config'), 'config',
                                              'panda',
                                              'srdf',
                                              'panda_arm.srdf.xacro')                                         
                                              
    srdf_config = Command(
        [FindExecutable(name='xacro'), ' ', srdf_file, ' hand:=true']
    )
    robot_description_semantic = {
        'robot_description_semantic': srdf_config
    }

    panda_zero_joints = {
      "zeros.fr3_joint4": -1.5708,
      "zeros.fr3_joint6": 1.5708 	
    }

    # Reach parameters
    reachability_yaml = load_yaml(
        "reachability_description", "config/panda/reachability_params.yaml"
    )
    reachability_params = {"reachability_params": reachability_yaml}


    rviz_base = os.path.join(get_package_share_directory("robots_config"), "rviz")
    rviz_full_config = os.path.join(rviz_base, "panda.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[robot_description]
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Joint State publisher
    joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[panda_zero_joints],
        output='screen')

    # Panda
    reach_gen = Node(
        package='reachability_description',
        executable='generate_reachability_node',
        output='screen',
        parameters=[reachability_params,
            robot_description,
            robot_description_semantic,
            {"chain_group_name": "fr3_manipulator"},
            {"robot_name": "fr3"},
            {"plugin_name": "reachability_description::ReachGraphReuleaux"}
        ]
    )    


    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            joint_publisher,
            reach_gen
        ]

    )
