#!/usr/bin/env python3
from collections import deque
import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


# ----- Joint Specs -----
JOINT_SPECS = {
    "min": np.deg2rad([-26.92, -11.22, -9.97, -13.0,-26.92, -11.22, -9.97, -13.0,-26.92, -11.22, -9.97, -13.0, -1, -10, -10.8, -5]),
    "max": np.deg2rad([ 26.92,  92.25, 97.92,  92.7, 26.92,  92.25, 97.92,  92.7, 26.92,  92.25, 97.92,  92.7, 85,  72,  94.0, 98]),
    "dir": np.array([-1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1])
}


# ----- Joint Map -----
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


# ----- Global Variables -----
TARGET_UPDATED = False
TARGET = deque(maxlen=1)
TARGET.append(np.zeros(16))


# ----- Callback -----
def cyberglove_callback(data):
    global TARGET, TARGET_UPDATED
    pos = [data.position[data.name.index(JOINT_MAP[i])] for i in range(16)] * JOINT_SPECS['dir']
    TARGET.append(np.minimum(JOINT_SPECS['max'], np.maximum(JOINT_SPECS['min'], pos)))


# ----- Main Script -----
if __name__ == '__main__':
    rospy.init_node('gazebo_cyberglove_cmd', anonymous=True)
    rate = rospy.Rate(30)
    
    ns = rospy.get_namespace()
    # ns = "/allegro_hand_right/"
    rospy.Subscriber(
        "/cyberglove/joint_states",
        JointState,
        cyberglove_callback,
        queue_size=1
    )

    publisher = rospy.Publisher(ns + "PositionController/command", Float64MultiArray, queue_size=1)

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = np.mean(TARGET, axis=0)
        publisher.publish(msg)
        rate.sleep()
