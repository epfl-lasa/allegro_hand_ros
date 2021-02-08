#!/usr/bin/env python3
import os
from collections import deque

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from termcolor import colored
from pynput import keyboard


# ----- Global Variables -----
class Global:
    messages = deque(maxlen=1)  # only store latest sensor value
    done_recording = False
    to_record = False
    data = []
    target = []


# ----- Callbacks -----
def cyberglove_callback(data):
    Global.messages.append(data)


def keyboard_callback(todo):
    if todo == "record":
        Global.to_record = True
    elif todo == "stop":
        Global.done_recording = True


# ----- Helper Functions -----
def write_to_file(filename, names, positions):
    print(colored("Writing to {:s}...".format(filename), "cyan"))
    with open(filename, "w") as f:
        f.write('\n'.join([
            '{:s} {:.6f}'.format(n, p)
            for n, p in zip(names, positions)
        ]))


def record_data():
    """Record data for callibration"""
    current_target = 0
    rospy.loginfo(
        "Press %s to record the point or %s to stop...",
        colored("<F2>", "yellow"),
        colored("<esc>", "red"),
    )
    while not Global.done_recording:
        if Global.to_record:
            try:
                msg = Global.messages.pop()
                Global.data.append(np.array(msg.position))
                current_target = current_target + 1
                rospy.loginfo("%i - %s", current_target, colored("Done!", "green"))
                Global.to_record = False
            except IndexError:
                # No message yet from the sensor
                pass

    rospy.loginfo(colored("Saving file...", "green"))
    np.savez(
        os.path.join(os.path.dirname(__file__), 'test_xyz_thumb.npz'),
        data=np.array(Global.data),
    )


# ----- Main function -----
def main():
    rospy.init_node('calibration_listener', anonymous=True)
    rospy.Subscriber("/allegro_hand_right/joint_states", JointState, cyberglove_callback)

    # Keyboard Listener
    listener = keyboard.GlobalHotKeys({
        '<f2>': lambda: keyboard_callback("record"),
        '<esc>': lambda: keyboard_callback("stop"),
    })
    listener.start()

    rospy.loginfo(colored(
        "Please put your fingers in as close configuration as shown in rviz.",
        "yellow"
    ))
    record_data()


if __name__ == '__main__':
    main()
