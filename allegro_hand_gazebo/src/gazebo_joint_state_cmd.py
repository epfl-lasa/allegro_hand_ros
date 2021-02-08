#!/usr/bin/env python3
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
