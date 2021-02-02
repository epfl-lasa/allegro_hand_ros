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
}

# ----- Global Variables -----
TARGET_UPDATED = False
TARGET = deque(maxlen=1)
TARGET.append(np.zeros(16))


# ----- Callback -----
def joint_state_callback(data):
    global TARGET
    TARGET.append(np.minimum(JOINT_SPECS['max'], np.maximum(JOINT_SPECS['min'], data.position)))


# ----- Main Script -----
if __name__ == '__main__':
    rospy.init_node('gazebo_joint_state_cmd', anonymous=True)
    rate = rospy.Rate(30)
    
    ns = rospy.get_namespace()
    rospy.Subscriber(
        "joint_cmd",
        JointState,
        joint_state_callback,
        queue_size=1
    )

    publisher = rospy.Publisher(ns + "PositionController/command", Float64MultiArray, queue_size=1)

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = np.mean(TARGET, axis=0)
        publisher.publish(msg)
        rate.sleep()
