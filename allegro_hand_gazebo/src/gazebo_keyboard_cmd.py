#!/usr/bin/env python3
#
# Copyright (C) 2021 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
# Authors:
# 	Vaibhav Gupta (vaibhav.gupta@epfl.ch)
# 	Saurav Aryan (saurav.aryan@epfl.ch) [Maintainer]
# Website: lasa.epfl.ch
#
# This file is part of `allegro_hand_gazebo`.
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
import rospy
from std_msgs.msg import String, Float64MultiArray


# ----- Joint Values (predefined) -----
JOINT_VALUES = {
    "pdControl": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "ready": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],     # Ready Pose
    "grasp_3": [-0.2, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.3, 0.4, 0.7, 0.9],  # 3 finger grasp
    "grasp_4": [-0.2, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.2, 1.0, 1.0, 1.0, 1.3, 0.6, 0.7, 0.9],  # 4 finger grasp
    "pinch_it": [0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.4, 0.0, 0.6, 0.8],  # Index finger pinch
    "pinch_mt": [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.3, 0.6, 0.7, 0.9],  # Middle finger pinch
    "envelop": [0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.4, 0.0, 0.6, 0.8],  # Envelop
    "gravcomp": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "off": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "save": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

# ----- Global Variables -----
TARGET = JOINT_VALUES["home"]


# ----- Callbacks -----
def keyboard_callback(data):
    """Callback for keyboard commands

    Args:
        data (String): ROS `String` message
    """
    global TARGET
    # Check if known target, otherwise go to home
    TARGET = JOINT_VALUES.get(data.data, JOINT_VALUES["home"])


# ----- Main Script -----
if __name__ == '__main__':
    rospy.init_node('gazebo_keyboard_cmd', anonymous=True)
    rate = rospy.Rate(30)

    rospy.Subscriber(
        "lib_cmd",
        String,
        keyboard_callback,
        queue_size=1,
    )

    publisher = rospy.Publisher(
        "PositionController/command",
        Float64MultiArray,
        queue_size=1,
    )

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = TARGET
        publisher.publish(msg)
        TARGET_UPDATED = False
        rate.sleep()
