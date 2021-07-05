# LOADING_MOTOR_DT

This README will briefly guide you to install and run the ROS2 version of *loading_motor_tb* for the project **PSG-453**
## General Overview
****

loading_motor_tb repo is a digital twin in ROS2 for the loading motor ABB 3GAA132214-ADE. 

It consists of 2 parts:
- ROS package tb_digital_twin
- Unity project containing vizualization

In ROS1, these 2 parts are interconnected and are communicating over the ROS bridge, however, they are also capable of running on their own, if you provide the required data (e.g. csv files / rostopic pub). In ROS2, we are still in the process if testing the new ROS HTTP Bridge.

In current version, we simulate the data reception with use of .csv files that are processed in ROS. This data is then used for computation and simulation purposes.

**NB!** In this repo you **WILL NOT** find the .csv files, as they are too big. Refer to the Requirements section of this README to learn how to retrieve them. 

## Requirements
****

1. You need to have a Virtual Machine(VM)/Machine with Linux Ubuntu (or equivalent) of at least ver. 18.04

2. You need to have Unity installed (preferably in Windows environment)

3. You need to have an established two-way connection between the VM/machine and host PC

4. You need to have the .csv files required to run the project( can be found [here](https://livettu.sharepoint.com/:f:/s/PSG453PUTprojectgroup/EiC93gX70itHoPBO5sS3aMMBApxqi6LMp3AXtNC7x-fKPA?e=0xOOCw) )

5. You need to build the [digital_twin_msgs](https://github.com/TalTech-PSG453/digital_twin_msgs) package

### ROS requirements

You need to have ROS2 installed of version Foxy Fitzroy

## How To Setup

### ROS package setup

1. Copy tb_digital_twin in the /src folder of your ROS2 workspace

2. Build first the *digital_twin_msgs* package.

3. Build the *tb_loading_motor* package

4. Place the downloaded files from **Requirements** section and extract them.

6. In file *params.yaml* in */config* folder replace the **filename** with the full path to the files you want to simulate
## How To Run
****
### ROS

Launch the **loading_motor_launch.py** file.

```
ros2 launch loading_motor_launch
```