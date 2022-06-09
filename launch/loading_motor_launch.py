import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():

    ns = LaunchConfiguration('namespace', default = 'tb_lm_left')
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('loading_motor_dt'),
        'config',
        'params.yaml'
    )

    current_simulator=Node(
        package = 'loading_motor_dt',
        name = 'current_simulator',
        executable = 'current_simulator',
        parameters = [config]
    )

    efficiency_map=Node(
        package = 'loading_motor_dt',
        name = 'efficiency_map',
        executable = 'efficiency_map',
        parameters = [config],
        remappings=[("torque", "electrical_torque_ref")]

    )
    torque_calculator=Node(
        package = 'loading_motor_dt',
        name = 'torque_calculator',
        executable = 'torque_calculator'
    )

    power_calculator=Node(
        package = 'loading_motor_dt',
        name = 'power_calculator',
        executable = 'power_calculator',
        parameters = [config]
    )

    angular_converter=Node(
        package = 'loading_motor_dt',
        name = 'angular_converter',
        executable = 'angular_converter'
    )

    nodes_with_ns = GroupAction(
        actions=[
            PushRosNamespace(ns),
            current_simulator,
            angular_converter,
            efficiency_map,
            power_calculator,
            torque_calculator

        ]
    )

    ld.add_action(nodes_with_ns)
    return ld