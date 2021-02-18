# allegro-hand unofficial fork

<!-- [![Build Status](https://travis-ci.org/felixduvallet/allegro-hand-ros.svg?branch=master)](https://travis-ci.org/felixduvallet/allegro-hand-ros) -->

This is an unofficial fork of SimLab's allegro [hand ros package](https://github.com/simlabrobotics/allegro_hand_ros).

It improves significantly upon the SimLab version by providing a catkin-ized version, simplifies the launch file structure, updates the package/node names to have a more consistent structure, improves the build process by creating a common driver, introduces an AllegroNode C++ class that reduces the amount of duplicated code. It also provides a python library that can control the hand directly.

It also provides the BHand library directly in this package (including both 32-bit and 64-bit versions, though 32-bit systems will need to update the symlink manually).

At this point no effort has been made to be backwards compatible.

## Launch file instructions
There is a single file (for realworld) to start any hand, [allegro_hand.launch](allegro_hand/launch/allegro_hand.launch), that starts the hand. It takes many arguments, but at a minimum you must specify the handedness:
```bash
roslaunch allegro_hand allegro_hand.launch HAND:=right
```

You can also simulate the hand very easily:
```bash
roslaunch allegro_hand allegro_hand.launch HAND:=right CONTROLLER:=sim
```    

Optional (recommended) arguments:
```
    ZEROS:=/path/to/zeros_file.yaml
    CONTROLLER:=grasp|pd|velsat|torque|sim
    RESPAWN:=true|false   Respawn controller if it dies.
    KEYBOARD:=true|false  (default is true)
    AUTO_CAN:=true|false  (default is true)
    CAN_DEVICE:=/dev/pcanusb1 | /dev/pcanusbNNN  (ls -l /dev/pcan* to see open CAN devices)
    VISUALIZE:=true|false  (Launch rviz)
    JSP_GUI:=true|false  (show the joint_state_publisher for *desired* joint angles)
```

Note on `AUTO_CAN`: There is a nice script `detect_pcan.py` which automatically finds an open `/dev/pcanusb` file. If instead you specify the can device manually (`CAN_DEVICE:=/dev/pcanusbN`), make sure you *also* specify `AUTO_CAN:=false`. Obviously, automatic detection cannot work with two hands.

The second launch file is for visualization, it is included in `allegro_hand.launch` if `VISUALIZE:=true`. Otherwise, it can be useful to run it separately (with `VISUALIZE:=false`), for example if you want to start rviz separately (and keep it running):
```bash
roslaunch allegro_hand allegro_viz.launch HAND:=right
```

Note that you should also specify the hand `robot_name` parameter in the viz launch if multiple handsare being used.

## Packages
 * **allegro_hand** A python client that enables direct control of the hand in python code, and all generic launch files.
 * **allegro_hand_driver** Driver for talking with the allegro hand.
 * **allegro_hand_controllers** Different nodes that actually control the hand. 
 The AllegroNode class handles all the generic driver comms, each class then implements `computeDesiredTorque` differently (and can have various topic subscribers):
   * grasp: Apply various pre-defined grasps, including gravity compensation.
   * pd: Joint space control: save and hold positions.
   * velsat: velocity saturation joint space control (supposedly experimental)
   * torque: Direct torque control.
   * sim: Just pass desired joint states through as current joint states.
 * **allegro_hand_gazebo** Gazebo simulation of allegro hand
 * **allegro_hand_cyberglove** Cyberglove controller for allegro hand
 * **allegro_hand_description** xacro descriptions for the kinematics of the hand, rviz configuration and meshes.
 * **allegro_hand_keyboard** Node that sends the commanded grasps. All commands are available with the grasp controller, only some are available with the other controllers.
 * **allegro_hand_parameters** All necessary parameters for loading the hand:
   * gains_pd.yaml: Controller gains for PD controller.
   * gains_velSat.yaml: Controller gains and parameters for velocity saturation controller.
   * initial_position.yaml: Home position for the hand.
   * zero.yaml: Offset and servo directions for each of the 16 joints, and some meta information about the hand.
   * zero_files/ Zero files for all hands.
 * **bhand** Library files for the predefined grasps, available in 32 and 64 bit versions. 64 bit by default, update symlink for 32 bit.

Note on polling (from SimLabs): The preferred sampling method is utilizing the Hand's own real time clock running @ 333Hz by polling the CAN communication (polling = true, default). In fact, ROS's interrupt/sleep combination might cause instability in CAN communication resulting unstable hand motions.

## Useful Links
 * [Allegro Hand wiki](http://www.simlab.co.kr/AllegroHand/wiki).
 * [ROS wiki for original package](http://www.ros.org/wiki/allegro_hand_ros).

## Installing the PCAN driver 
Before using the hand, you must install the pcan drivers. This assumes you have a peak-systems pcan to usb adapter. This step might not be needed in modern version of the OS.

1. Install these packages
    ```bash
    sudo apt-get install libpopt-dev ros-indigo-libpcan
    ```

2. Download latest drivers from http://www.peak-system.com/fileadmin/media/linux/index.htm#download

3. Install the drivers:
    ```bash
    make clean; make NET=NO_NETDEV_SUPPORT
    sudo make install
    sudo /sbin/modprobe pcan
    ```

4. Test that the interface is installed properly with:
    ```bash
    cat /proc/pcan
    ```
    You should see some stuff streaming.

5. When the hand is connected, you should see pcanusb0 or pcanusb1 in the list of available interfaces:
    ```bash
    ls -l /dev/pcan*
    ```

    If you do not see any available files, you may need to run the following from the downloaded pcan folder: this theoretically creates the devices files if the system has not done it automatically.
    ```bash
    sudo ./driver/pcan_make_devices 2
    ```

## Gazebo ~~alpha~~ support
Gazebo is *now* supported ~~in theory~~, thanks to inertial parameters from @trhermans.
~~However, I have not been successful at running a physical simulation of the robot hand. I am leaving the gazebo option for others to experiment; to try it out, pass GAZEBO:=true to the launch file.~~

~~Contributions (pull requests) to fix the simulation are welcome.~~

Gazebo simulation can be run using
```bash
roslaunch allegro_hand_gazebo allegro_hand_gazebo.launch
```
More information can be found in the [README](allegro_hand_gazebo/README.md) of `allegro_hand_gazebo` package.
