# LOADING_MOTOR_DT

This README will briefly guide you to install and run the ROS2 version of *loading_motor_tb* for the project [**PSG-453**](https://www.etis.ee/Portal/Projects/Display/72b66c74-e911-49c3-ac6a-6716f9e72ba5?lang=ENG)
## General Overview
****

loading_motor_tb repo is a digital twin in ROS2 for the *ABB 3GAA132214-ADE* loading motor.

It consists of 2 parts:
- ROS package tb_digital_twin
- Unity project containing vizualization

In ROS1, these 2 parts are interconnected and are communicating over the ROS bridge, however, they are also capable of running on their own, if you provide the required data (e.g. csv files / rostopic pub). In ROS2, we are still in the process if testing the new ROS HTTP Bridge.

At the moment, we simulate the data reception with use of .csv files that are processed in ROS. This data is then used for computation and simulation purposes. In the nearest future we will work on the hardware bridge connecting the real motor and the digital twin to allow real-time data exchange.

**NB!** In this repo you **WILL NOT** find the .csv files for simulations, as they are too large in size. Refer to the Requirements section of this README to learn how to retrieve them.

## Requirements
****

1. You need to have a Virtual Machine(VM)/Machine with Linux Ubuntu (or equivalent) of at least ver. 18.04

2. You need to have Unity installed (tested only in Windows environment)

3. You need to have an established two-way connection between the VM/machine and host PC (where the Unity will be running)

4. You need to have the .csv files required to run the project( can be found [here](https://livettu.sharepoint.com/:f:/s/PSG453PUTprojectgroup/EiC93gX70itHoPBO5sS3aMMBApxqi6LMp3AXtNC7x-fKPA?e=0xOOCw) )

5. You need to build the [digital_twin_msgs](https://github.com/TalTech-PSG453/digital_twin_msgs) package before compiling this package.

### ROS requirements

You need to have ROS2 installed of version Foxy Fitzroy

## How To Setup

### ROS package setup

1. Copy the *tb_digital_twin* in the /src folder of your ROS2 workspace

2. Build first the *digital_twin_msgs* package.
```
colcon build --packages-select digital_twin_msgs
```
3. Place the downloaded files from **Requirements** section and extract them.

4. In file *params.yaml* in */config* folder replace the **filename** with the full path to the files you want to simulate

5. Build the *tb_loading_motor* package
```
colcon build --packages-select loading_motor_dt
```

## How To Run
****
### ROS2

Launch the **loading_motor_launch.py** file.

```
ros2 launch loading_motor_dt loading_motor_launch.py
```

## Running the windings checker

### Overview

Windings analyzer is a script that checks the 3-phase current of the Loading Motor and compares it to each other to spot potential malfunctions. If deviations are spotted by the checker, the script calls Matlab to perform Park & Clarke transform analysis on the recorded 3-phase current. The output is then retrieved and is published in ROS2 as D-and-Q axis currents.

### Requirements

1. You need to have Matlab with active license installed on the same machine as ROS2.

2. You need to have *parkandclarke.m* included in Matlab path.

3. In file *params.yaml* in */config* folder replace the **matlab_path** with the full path to *parkandclarke.m*

4. Build the *tb_loading_motor* package
```
colcon build --packages-select loading_motor_dt
```

## Running

Launch the script with the following command:

```
ros2 launch loading_motor_dt windings_checker_launch.py
```

# Publications 

1. [Digital Twin Service Unit for AC Motor Stator Inter-Turn Short Circuit Fault Detection](10.1109/IWED52055.2021.9376328)

2. [ROS middle-layer integration to Unity 3D as an interface option for propulsion drive simulations of autonomous vehicles](10.1088/1757-899X/1140/1/012008)