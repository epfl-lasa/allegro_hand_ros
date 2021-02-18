# Gazebo simulation of allegro hand
ROS package for the Gazebo simulation of allegro hand.

## Demos

## Run Instructions
```bash
roslaunch allegro_hand_gazebo allegro_hand_gazebo.launch
```

## Launch files
1. [`allegro_hand_gazebo.launch`](launch/allegro_hand_gazebo.launch)
This is the main launch file to launch gazebo simulation.
Arguments:
    - `hand` (str) {needed} &rightarrow; One of {'right', 'left'},  currently only 'right' is supported
    - `visualize` (bool) &rightarrow; Run rviz visualization
    - `robot_name` (str) &rightarrow; Name for allegro hand
    - `controller` (str) &rightarrow; Controller for Gazebo, one of {'TorqueController', 'PositionTorqueController', 'PositionController'}
    - `gzclient` (bool) &rightarrow; Run in gazebo client mode
    - `keyboard` (bool) &rightarrow; Use keyboard for allegro hand control
    - `cyberglove` (bool) &rightarrow; Use cyberglove for allegro hand control
    - `global_z` (bool) &rightarrow; Global z-offset during spawn (Useful when attaching to other robots)
    - `hardware_interface` (bool) &rightarrow; Hardware interface to be used (selected automatically from `controller`)

## Authors/Maintainers
- [Vaibhav Gupta](https://github.com/guptavaibhav0)
