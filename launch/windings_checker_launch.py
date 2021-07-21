import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('loading_motor_dt'),
        'config',
        'params.yaml'
        )
        
    data_processor=Node(
        package = 'loading_motor_dt',
        namespace = 'tb_lm',
        name = 'windings_checker',
        executable = 'winding_error_checker.py',
        parameters = [config]
    )
    ld.add_action(data_processor)
    return ld