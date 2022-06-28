import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from rewritten_yaml import RewrittenYaml

import yaml
from pathlib import Path
home = str(Path.home())

def prepend_to_filenames(config_file, package_name):

    with open(config_file, "r") as f:
        config_yaml = yaml.safe_load(f)

    for ns in config_yaml:
        for node in config_yaml[ns]:
            try:
                filename = config_yaml[ns][node]["ros__parameters"]["filename"]
                config_yaml[ns][node]["ros__parameters"]["filename"] = os.path.join(
                    get_package_share_directory(package_name),'data_files', filename)
            except:
                pass
    with open(os.path.join(home,"debug.yaml"),'w') as f:
        yaml.dump(config_yaml, f)

    return config_yaml

def generate_launch_description():

    ns = LaunchConfiguration('namespace', default = 'tb_tm')
    
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('loading_motor_dt'),
        'config',
        'params.yaml'
    )
    params_prepended = prepend_to_filenames(config, "loading_motor_dt")
    
    configured_params = RewrittenYaml(
        source_file=config,
        root_key='tb_tm',
        param_rewrites=params_prepended
        )    

    current_simulator=Node(
        package = 'loading_motor_dt',
        name = 'current_simulator',
        executable = 'current_simulator',
        parameters = [configured_params]
    )

    current_simulator_with_ns = GroupAction(
        actions=[
            PushRosNamespace(ns),
            current_simulator,
        ]
    )

    ld.add_action(current_simulator_with_ns)
    return ld
