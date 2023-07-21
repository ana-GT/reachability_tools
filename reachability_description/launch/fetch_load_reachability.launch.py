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

def generate_launch_description():

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("robots_config"),
            "robots", "fetch",
            "fetch.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    srdf_file = os.path.join(get_package_share_directory('robots_config'), 'config',
                                     'fetch', 'fetch.srdf')
    srdf_config = open(srdf_file).read()

    robot_description_semantic = {
        'robot_description_semantic': srdf_config
    }


    rviz_base = os.path.join(get_package_share_directory("robots_config"), "rviz")
    rviz_full_config = os.path.join(rviz_base, "fetch.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[]
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
        output='screen')

    # Panda
    load_reach = Node(
        package='reachability_description',
        executable='load_reachability_node',
        output='screen',
        parameters=[{
            "robot_description": robot_description_config.toxml(),
            "robot_description_semantic" : srdf_config,
            "chain_group_name": "arm_with_torso", # arm
            "robot_name": "fetch" 
        }]
        )    


    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            joint_publisher,
            load_reach
        ]

    )
