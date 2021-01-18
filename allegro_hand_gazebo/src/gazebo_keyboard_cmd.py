#!/usr/bin/env python3

import rospy

import math
import numpy as np
from scipy import interpolate

from std_msgs.msg import String, Float64MultiArray
from gazebo_msgs.msg import ContactsState


# ----- Joint Values -----
JOINT_VALUES = {
    "pdControl" : [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home"      : [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "ready"     : [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "grasp_3"   : [-0.2, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.3, 0.4, 0.7, 0.9], # 3 finger grasp
    "grasp_4"   : [-0.2, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.2, 1.0, 1.0, 1.0, 1.3, 0.6, 0.7, 0.9], # 4 finger grasp
    "pinch_it"  : [ 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.4, 0.0, 0.6, 0.8], # Index finger pinch
    "pinch_mt"  : [ 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.3, 0.6, 0.7, 0.9], # Middle finger pinch
    "envelop"   : [ 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.4, 0.0, 0.6, 0.8], # Envelop
    "gravcomp"  : [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "off"       : [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "save"      : [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

# ----- Global Variables -----
MOVING = False
TARGET_UPDATED = False
TARGET = JOINT_VALUES["home"]


# ----- Interpolation Functions -----
def slerp(v0, v1, t_array):
    """Spherical linear interpolation."""
    # >>> slerp([1,0,0,0], [0,0,0,1], np.arange(0, 1, 0.001))
    t_array = np.array(t_array)
    v0 = np.array(v0)
    v1 = np.array(v1)
    dot = np.sum(v0 * v1)

    if dot < 0.0:
        v1 = -v1
        dot = -dot
    
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        result = v0[np.newaxis,:] + t_array[:,np.newaxis] * (v1 - v0)[np.newaxis,:]
        return (result.T / np.linalg.norm(result, axis=1)).T
    
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)

    theta = theta_0 * t_array
    sin_theta = np.sin(theta)
    
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0[:,np.newaxis] * v0[np.newaxis,:]) + (s1[:,np.newaxis] * v1[np.newaxis,:])


def interpolate(x0, x1, t_array):
    """Interpolation of Position and Orientation"""
    position_interp = interpolate.interp1d(x0, x1)
    positions = position_interp(t_array)
    orientations = slerp(v0, v1, t_array)
    pass


def IK_solver():
    pass


# ----- Callback -----
def keyboard_callback(data):
    global TARGET_UPDATED, TARGET
    if ~MOVING:
        TARGET_UPDATED = True
        TARGET = JOINT_VALUES.get(data.data, JOINT_VALUES["home"])

# ----- Main Script -----
if __name__ == '__main__':
    rospy.init_node('gazebo_keyboard_cmd', anonymous=True)
    rate = rospy.Rate(30)
    
    ns = rospy.get_namespace()
    # ns = "/allegro_hand_right/"
    rospy.Subscriber(
        ns + "lib_cmd",
        String,
        keyboard_callback,
        queue_size=1
    )

    publisher = rospy.Publisher(ns+"PositionController/command", Float64MultiArray, queue_size=1)

    while not rospy.is_shutdown():
        if TARGET_UPDATED:
            msg = Float64MultiArray()
            msg.data = TARGET
            publisher.publish(msg)
            TARGET_UPDATED = False            
        rate.sleep()
