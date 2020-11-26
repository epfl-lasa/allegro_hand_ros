#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import math
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import String
from gazebo_msgs.msg import ContactsState


links = [
    "palm",
    "finger_0/link_0",
    "finger_0/link_1",
    "finger_0/link_2",
    "finger_0/link_3",
    "finger_0/link_tip",
    "finger_1/link_0",
    "finger_1/link_1",
    "finger_1/link_2",
    "finger_1/link_3",
    "finger_1/link_tip",
    "finger_2/link_0",
    "finger_2/link_1",
    "finger_2/link_2",
    "finger_2/link_3",
    "finger_2/link_tip",
    "finger_3/link_0",
    "finger_3/link_1",
    "finger_3/link_2",
    "finger_3/link_3",
    "finger_3/link_tip",
]


class BasicMarker():
    def __init__(self, frame, ns, unique_id):
        self._cmap = plt.cm.get_cmap('autumn_r')
        self._norm = plt.Normalize(vmin=0.0, vmax=5.0, clip=True)

        self.marker_object = Marker()
        self.marker_object.type = Marker.ARROW
        self.marker_object.action = Marker.ADD
        self.marker_object.ns = ns
        self.marker_object.id = unique_id
        self.marker_object.header.frame_id = "world"#frame
        self.marker_object.frame_locked = True
        self._set_scale()
        self._set_colour()
        self._set_points([[0,0,0], [0,0,0]])
        self.updated = True

        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0.1)

    def _set_points(self, points):
        self.marker_object.points = []
        for [x, y, z] in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            self.marker_object.points.append(point)

    def _set_colour(self, pressure=0.0):
        rgba = self._cmap(self._norm(pressure))
        self.marker_object.color.r = rgba[0]
        self.marker_object.color.g = rgba[1]
        self.marker_object.color.b = rgba[2]
        self.marker_object.color.a = rgba[3]

    def _set_scale(self,
                   diameter=0.005,
                   head_diameter=0.008,
                   head_length=0.0):
        self.marker_object.scale.x = diameter
        self.marker_object.scale.y = head_diameter
        self.marker_object.scale.z = head_length

    def update_marker(self, p1, p2, pressure):
        if not self.updated:
            self.marker_object.header.stamp = rospy.get_rostime()
            self._set_points([p1, p2])
            self._set_colour(np.abs(pressure)) 
            self.updated = True

    def get_marker(self):
        self.updated = False
        return self.marker_object


class ContactsDisplay():
    def __init__(self, ns, links):
        self.markers = [
            BasicMarker(frame=links[i], ns=ns, unique_id=i)
            for i in range(len(links))
        ]
        self.subscribers = [
            rospy.Subscriber(
                ns + links[i] + "/contact",
                ContactsState,
                self.contact_callback,
                callback_args=i,
                queue_size=1
            )
            for i in range(len(links))
        ]
        self.publisher = rospy.Publisher("rviz_gazebo_contact", MarkerArray, queue_size=1)

    def publish(self):
        markerarray_object = MarkerArray()
        markerarray_object.markers = [
            marker.get_marker()
            for marker in self.markers
        ]
        self.publisher.publish(markerarray_object)

    def contact_callback(self, data, ii):
        n = len(data.states)
        if n == 0:
            self.markers[ii].update_marker([0, 0, 0], [0, 0, 0], 0.0)
            return
        
        if n > 0:
            n_contacts = len(data.states[0].contact_positions)

            weighted_P = np.zeros(3,)
            weighted_N = np.zeros(3,)
            total_normal_force = 0

            for i in range(n_contacts):
                P = np.asarray([
                    data.states[0].contact_positions[i].x,
                    data.states[0].contact_positions[i].y,
                    data.states[0].contact_positions[i].z,
                ])
                N = np.asarray([
                    data.states[0].contact_normals[i].x,
                    data.states[0].contact_normals[i].y,
                    data.states[0].contact_normals[i].z,
                ])
                F = np.asarray([
                    data.states[0].wrenches[i].force.x,
                    data.states[0].wrenches[i].force.y,
                    data.states[0].wrenches[i].force.z,
                ])
                normal_force = np.dot(N, F)
                if normal_force > 0:
                    # only pressure forces
                    weighted_P += normal_force * P
                    weighted_N += normal_force * N
                    total_normal_force += normal_force

            P = weighted_P / total_normal_force
            N = weighted_N / total_normal_force

            F = np.asarray([
                data.states[0].total_wrench.force.x,
                data.states[0].total_wrench.force.y,
                data.states[0].total_wrench.force.z,
            ])
            pressure = total_normal_force# np.dot(F, N)

            temp = P + np.sign(pressure) * N * 0.05
            self.markers[ii].update_marker(P, temp, pressure)

if __name__ == '__main__':
    rospy.init_node('rviz_gazebo_contacts', anonymous=True)
    rate = rospy.Rate(30)
    
    # ns = rospy.get_namespace()
    ns = "/allegro_hand_right/"
    contact_display = ContactsDisplay(ns, links)

    while not rospy.is_shutdown():
        contact_display.publish()
        rate.sleep()
