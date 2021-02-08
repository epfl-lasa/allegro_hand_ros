#!/usr/bin/env python3
import os
from collections import deque
import numpy as np

import rospy
from sensor_msgs.msg import JointState

import joblib


# ----- Global Variables -----
TARGET = deque(maxlen=1)    # Change `maxlen` for window averaging
TARGET.append(np.zeros(16))
ML_MODEL = None

# ----- Joint Specs, Map and Name -----
JOINT_SPECS = {
    "min": np.deg2rad([
        -26.92, -11.22, -9.97, -13.0,   # finger 1
        -26.92, -11.22, -9.97, -13.0,   # finger 2
        -26.92, -11.22, -9.97, -13.0,   # finger 3
        -1.00, -10.00, -10.80, -5.00,   # thumb
    ]),
    "max": np.deg2rad([
        26.92, 92.25, 97.92, 92.70,     # finger 1
        26.92, 92.25, 97.92, 92.70,     # finger 2
        26.92, 92.25, 97.92, 92.70,     # finger 3
        85.00, 72.00, 94.00, 98.00,     # thumb
    ]),
    # Direction change (if needed)
    "dir": np.array([
        -1, +1, +1, +1,     # finger 1
        -1, +1, +1, +1,     # finger 2
        -1, +1, +1, +1,     # finger 3
        +1, +1, +1, +1,     # thumb
    ])
}
JOINT_MAP = [
    "index_abduction_joint",
    "index_proximal_joint",
    "index_middle_joint",
    "index_distal_joint",
    "middle_abduction_joint",
    "middle_proximal_joint",
    "middle_middle_joint",
    "middle_distal_joint",
    "ring_abduction_joint",
    "ring_proximal_joint",
    "ring_middle_joint",
    "ring_distal_joint",
    "thumb_abduction_joint",
    "thumb_proximal_joint",
    "thumb_middle_joint",
    "thumb_distal_joint",
]
JOINT_NAME = [
    "finger_0/joint_0",
    "finger_0/joint_1",
    "finger_0/joint_2",
    "finger_0/joint_3",
    "finger_1/joint_0",
    "finger_1/joint_1",
    "finger_1/joint_2",
    "finger_1/joint_3",
    "finger_2/joint_0",
    "finger_2/joint_1",
    "finger_2/joint_2",
    "finger_2/joint_3",
    "finger_3/joint_0",
    "finger_3/joint_1",
    "finger_3/joint_2",
    "finger_3/joint_3",
]


# ----- Callback -----
def cyberglove_callback(data):
    """Callback for `JointState` message from Cyberglove
    Remap cyberglove joint positions to Allegro Hand joint positions and
    updates target to `JointState` while ensuring joint limits.

    Args:
        data (JointState): ROS `JointState` message from Cyberglove
    """
    global TARGET

    # Remap cyberglove joint positions to Allegro Hand joint positions
    pos = [
        data.position[data.name.index(JOINT_MAP[i])]
        for i in range(16)
    ] * JOINT_SPECS['dir']

    if ML_MODEL is not None:
        thumb_pos = np.concatenate([ML_MODEL[i].predict(np.array(pos[-4:]).reshape(1, -1)) for i in range(4)])
        pos = np.concatenate((pos[:-4], thumb_pos))

    TARGET.append(np.minimum(JOINT_SPECS['max'], np.maximum(JOINT_SPECS['min'], pos)))


# ----- Main Script -----
if __name__ == '__main__':
    rospy.init_node('gazebo_cyberglove_cmd', anonymous=True)
    rate = rospy.Rate(30)

    try:
        ML_MODEL = joblib.load(os.path.join(
            os.path.dirname(__file__),
            "..", "data",
            "trainedModel_GPR.joblib"
        ))
    except FileNotFoundError:
        # No ML Model found
        ML_MODEL = None

    rospy.Subscriber(
        "/cyberglove/joint_states",
        JointState,
        cyberglove_callback,
        queue_size=1
    )

    publisher = rospy.Publisher(
        "joint_cmd",
        JointState,
        queue_size=1
    )

    while not rospy.is_shutdown():
        msg = JointState()
        msg.name = JOINT_NAME
        msg.position = np.mean(TARGET, axis=0)
        publisher.publish(msg)
        rate.sleep()
