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
from collections import deque
import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


# ----- Joint Specs -----
JOINT_SPECS = {
    "min": np.deg2rad([
        -26.92, -11.22, -9.97, -13.0,
        -26.92, -11.22, -9.97, -13.0,
        -26.92, -11.22, -9.97, -13.0,
        -1.00, -10.00, -10.80, -5.00,
    ]),
    "max": np.deg2rad([
        26.92, 92.25, 97.92, 92.70,
        26.92, 92.25, 97.92, 92.70,
        26.92, 92.25, 97.92, 92.70,
        85.00, 72.00, 94.00, 98.00,
    ]),
}

# ----- Global Variables -----
TARGET_UPDATED = False
TARGET = deque(maxlen=1)    # Change `maxlen` for window averaging
TARGET.append(np.zeros(16))


# ----- Callbacks -----
def joint_state_callback(data):
    """Callback for `JointState` message
    Updates target to `JointState` while ensuring joint limits.

    Args:
        data (JointState): ROS `JointState` message
    """
    global TARGET
    TARGET.append(np.minimum(JOINT_SPECS['max'], np.maximum(JOINT_SPECS['min'], data.position)))


# ----- Main Script -----
if __name__ == '__main__':
    rospy.init_node('gazebo_joint_state_cmd', anonymous=True)
    rate = rospy.Rate(30)

    rospy.Subscriber(
        "joint_cmd",
        JointState,
        joint_state_callback,
        queue_size=1
    )

    publisher = rospy.Publisher(
        "PositionController/command",
        Float64MultiArray,
        queue_size=1
    )

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = np.mean(TARGET, axis=0)
        publisher.publish(msg)
        rate.sleep()
