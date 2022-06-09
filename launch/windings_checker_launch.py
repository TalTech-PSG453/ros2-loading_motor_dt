import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    
    ns = LaunchConfiguration('namespace', default = 'tb_lm_right')

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('loading_motor_dt'),
        'config',
        'params.yaml'
        )
        
    winding_checker=Node(
        package = 'loading_motor_dt',
        name = 'windings_checker',
        executable = 'winding_error_checker.py',
        parameters = [config]
    )

    winding_checker_ns = GroupAction(
        actions=[
            PushRosNamespace(ns),
            winding_checker,
        ]
    )    
    ld.add_action(winding_checker_ns)
    return ld