#!/usr/bin/env python3
#
# Copyright (C) 2021 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
# Authors:
# 	Vaibhav Gupta (vaibhav.gupta@epfl.ch)
# 	Saurav Aryan (saurav.aryan@epfl.ch) [Maintainer]
# Website: lasa.epfl.ch
#
# This file is part of `allegro_hand_cyberglove`.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
import os
from collections import deque

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from termcolor import colored
from pynput import keyboard


# ----- Global Variables -----
class Global:
    calibration_type = "base"   # {base, center, thumb}
    messages = deque(maxlen=1)  # only store latest sensor value
    done_recording = False
    to_record = False
    data = []
    target = []


# ----- Callbacks -----
def cyberglove_callback(data):
    """Save the messages from the cyberglove

    Args:
        data (JointState): message from cyberglove driver
    """
    Global.messages.append(data)


def keyboard_callback(todo):
    """Start or stop recording based on keyboard input

    Args:
        todo (str): One of {'record', 'stop'}
    """
    if todo == "record":
        Global.to_record = True
    elif todo == "stop":
        Global.done_recording = True


# ----- Helper Functions -----
def record_data():
    """Record data for callibration"""
    current_target = 0
    rospy.loginfo(
        "Press %s to record the point, or %s to stop and save data...",
        colored("<F2>", "yellow"),
        colored("<esc>", "red"),
    )
    while not Global.done_recording:
        if rospy.is_shutdown():
            # Quit if ROS is shutdown
            return

        if Global.to_record:
            try:
                # Store the data from message and reset `to_record` flag
                msg = Global.messages.pop()
                Global.data.append(np.array(msg.position))
                current_target = current_target + 1
                rospy.loginfo("%i - %s", current_target, colored("Done!", "green"))
                Global.to_record = False
            except IndexError:
                # No message yet from the sensor
                pass

    # Save the data to a file
    rospy.loginfo(colored("Saving file...", "green"))
    np.savez(
        os.path.join(os.path.dirname(__file__),
                     "allegro_{:s}.npz".format(Global.calibration_type)),
        data=np.array(Global.data),
    )


# ----- Main function -----
def main():
    """Main logic for calibration"""
    rospy.init_node('calibration_listener', anonymous=True)
    rospy.Subscriber("joint_states", JointState, cyberglove_callback)

    # Keyboard Listener
    listener = keyboard.GlobalHotKeys({
        '<f2>': lambda: keyboard_callback("record"),
        '<esc>': lambda: keyboard_callback("stop"),
    })
    listener.start()

    record_data()


if __name__ == '__main__':
    main()
