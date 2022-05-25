# LOADING_MOTOR_DT

This README will briefly guide you to install and run the ROS2 version of *loading_motor_tb* for the project [**PSG-453**](https://www.etis.ee/Portal/Projects/Display/72b66c74-e911-49c3-ac6a-6716f9e72ba5?lang=ENG)
## General Overview
****

Initially, loading_motor_tb repo is a digital twin in ROS2 for the *ABB 3GAA132214-ADE* loading motor. However, it is possible to use it 
for any other motor, depending on provided data.

The whole digital twin consists of 3 parts:
- ROS2 package tb_digital_twin
- Set of ROS2 software packages that are autogenerated C++ from MATLAB (serve as service entities)
- Unity project containing vizualization of the loading motor

## Usage

Clone this Git repo and compile it. The entry point of this loading motor digital twin is the current and voltage data.
This can be either simulated using the ParseDewetron class and a supplied *.csv* file, or it can be gathered and then supplied via ROS2 to appropriate topics (*/supply_input*).

Sample *.csv* data files can be found in this repo in the `/src/current_simulator/` folder. Alternatively, data files can be found [here](https://livettu.sharepoint.com/:f:/s/PSG453PUTprojectgroup/EiC93gX70itHoPBO5sS3aMMBApxqi6LMp3AXtNC7x-fKPA?e=0xOOCw). The `parse_dewetron.hpp` header file contains information on the expected format of the data. (In case you need to simulate the data yourself).

## Requirements
****

1. You need to have a Virtual Machine(VM)/Machine with Linux Ubuntu (or equivalent) of at least ver. 18.04

2. You need to have Unity installed (tested only in Windows environment)

3. You need to have an established two-way connection between the Linux machine (ROS2) and Windows machine (Unity)

4. You need to clone and build the [digital_twin_msgs](https://github.com/TalTech-PSG453/digital_twin_msgs) package before compiling this package.

5. You need to clone and build the [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) for communication with Unity over WebSockets. 

### ROS requirements

You need to have ROS2 installed of version Foxy Fitzroy

## How To Setup

### ROS package setup

1. Clone this repo in the /src folder of your ROS2 workspace

2. Build first the [digital_twin_msgs](https://github.com/TalTech-PSG453/digital_twin_msgs) package.
```
colcon build --packages-select digital_twin_msgs
```
3. In file *params.yaml* in */config* folder replace the **filename** with the full path to the files you want to simulate (make sure the names are correct and files are correctly specified)

4. Build the *loading_motor_dt* package
```
colcon build --packages-select loading_motor_dt
```
### winding_error_checker

Please refer to `/src/windings_error_checker` for instructions how to run MATLAB and ROS2 together and to check the malfunctioning of windings of the motor.

## How To Run
****
### ROS2

1. Firstly, navigate to the ROS2 workspace and launch the *rosbridge_server* to enable connection with Unity:

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 

```
2. There are several launch files available in `/launch` folder.

* To launch only one loading_motor use the **loading_motor_launch.py** file.

```
ros2 launch loading_motor_dt loading_motor_launch.py
```

* To launch two loading_motor instances, use the **loading_motors_all_launch.py**

```
ros2 launch loading_motor_dt loading_motor_launch.py
```

# Publications 

1. V. Rjabtšikov et al., "Digital Twin Service Unit for AC Motor Stator Inter-Turn Short Circuit Fault Detection," *2021 28th International Workshop on Electric Drives: Improving Reliability of Electric Drives (IWED)*, 2021, pp. 1-5, doi: [10.1109/IWED52055.2021.9376328](https://doi.org/10.1109/IWED52055.2021.9376328).

2. V. Kuts et al., ‘ROS middle-layer integration to Unity 3D as an interface option for propulsion drive simulations of autonomous vehicles’, *IOP Conference Series: Materials Science and Engineering*, vol. 1140, no.1, pp. 012008, May 2021, doi: [10.1088/1757-899X/1140/1/012008](https://doi.org/10.1088/1757-899X/1140/1/012008).

3. V. Kuts et al., "Digital Twin: Universal User Interface for Online Management of the Manufacturing System." *Proceedings of the ASME 2021 International Mechanical Engineering Congress and Exposition. Volume 2B: Advanced Manufacturing*. Virtual, Online. November 1–5, 2021. V02BT02A003. ASME. doi: [10.1115/IMECE2021-69092](https://doi.org/10.1115/IMECE2021-69092).