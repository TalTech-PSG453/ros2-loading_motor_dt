import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ld = LaunchDescription()
    
    package_prefix = get_package_share_directory('loading_motor_dt')
    loading_left = PythonLaunchDescriptionSource([package_prefix,'/launch/loading_motor_launch.py'])
    loading_right = PythonLaunchDescriptionSource([package_prefix,'/launch/loading_motor_launch.py'])

    launch_left = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            #PushRosNamespace('tb_lm_left'),
            IncludeLaunchDescription(loading_left, launch_arguments = {'namespace':'tb_lm_left'}.items()),
        ]
    )
    launch_right = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            #PushRosNamespace('tb_tm'),
            IncludeLaunchDescription(loading_right, launch_arguments = {'namespace':'tb_lm_right'}.items()),
        ]
    )

    ld.add_action(launch_right)
    ld.add_action(launch_left)
    return ld
