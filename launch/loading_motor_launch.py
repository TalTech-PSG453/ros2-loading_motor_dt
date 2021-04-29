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
        name = 'data_processor',
        executable = 'currentVoltageFlow',
        parameters = [config]
    )

    efficiency_map=Node(
        package = 'loading_motor_dt',
        namespace = 'tb_lm',
        name = 'efficiency_map',
        executable = 'efficiencyMap',
        parameters = [config],
        remappings=[
        ("torque", "electrical_torque_ref")
        ]
    )
    torque_calculator=Node(
        package = 'loading_motor_dt',
        namespace = 'tb_lm',
        name = 'torque_calculator',
        executable = 'torqueCalculator'
    )

    power_calculator=Node(
        package = 'loading_motor_dt',
        namespace = 'tb_lm',
        name = 'power_calculator',
        executable = 'powerCalculator'
    )

    angular_converter=Node(
        package = 'loading_motor_dt',
        namespace = 'tb_lm',
        name = 'angular_converter',
        executable = 'angularConverter'
    )
    ld.add_action(data_processor)
    ld.add_action(angular_converter)
    ld.add_action(efficiency_map)
    ld.add_action(power_calculator)
    ld.add_action(torque_calculator)
    return ld