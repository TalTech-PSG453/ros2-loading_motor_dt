import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('loading_motor_dt'),
        'config',
        'params.yaml'
        )
    
    params_prepended = prepend_to_filenames(config, "loading_motor_dt")

    data_processor=Node(
        package = 'loading_motor_dt',
        namespace = 'tb_lm_left',
        name = 'efficiency_map',
        executable = 'efficiency_map',
        parameters = [params_prepended]
    )
    ld.add_action(data_processor)
    return ld