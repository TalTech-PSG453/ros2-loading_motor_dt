import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():

    ns = LaunchConfiguration('namespace', default = 'tb_tm')
    
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('loading_motor_dt'),
        'config',
        'params.yaml'
    )

    data_processor=Node(
        package = 'loading_motor_dt',
        name = 'data_processor',
        executable = 'currentVoltageFlow',
        parameters = [config]
    )

    data_processor_with_ns = GroupAction(
        actions=[
            PushRosNamespace(ns),
            data_processor,
        ]
    )

    ld.add_action(data_processor_with_ns)
    return ld
