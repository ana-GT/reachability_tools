import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("robots_config"),
            "robots", "yumi",
            "yumi.urdf.xacro",
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


    yumi_zero_joints = { 
        "zeros": {
          "yumi_joint_1_l": 0, 
          "yumi_joint_2_l": -1.57, 
          "yumi_joint_7_l": 1.57, 
          "yumi_joint_3_l": 0.7, 
          "yumi_joint_4_l": -1.57, 
          "yumi_joint_5_l": 1.3, 
          "yumi_joint_6_l": 1.57, 
          "yumi_joint_1_r": 0, 
          "yumi_joint_2_r": -1.57, 
          "yumi_joint_7_r": -1.57, 
          "yumi_joint_3_r": 0.7, 
          "yumi_joint_4_r": -1.57, 
          "yumi_joint_5_r": -1.3, 
          "yumi_joint_6_r": -1.57        
       }
    }

    rviz_base = os.path.join(get_package_share_directory("robots_config"), "rviz")
    rviz_full_config = os.path.join(rviz_base, "yumi.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
        robot_description,
        robot_description_semantic
        ]
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
        parameters=[yumi_zero_joints],
        output='screen')

    # Estimate
    load_reach = Node(
        package='reachability_description',
        executable='load_reachability_node',
        output='screen',
        parameters=[
            reachability_params,
            robot_description,
            robot_description_semantic,
            {"chain_group_name": "right_arm"}, # right_arm
            {"robot_name": "yumi"} 
        ]
    ) 

    return LaunchDescription(
        [
            rviz_node,
#            static_tf,
            robot_state_publisher,
            joint_publisher,
            load_reach
        ]

    )
