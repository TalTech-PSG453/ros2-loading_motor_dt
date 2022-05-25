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
