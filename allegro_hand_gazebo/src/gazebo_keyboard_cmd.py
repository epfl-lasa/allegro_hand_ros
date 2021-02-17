#!/usr/bin/env python3
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
