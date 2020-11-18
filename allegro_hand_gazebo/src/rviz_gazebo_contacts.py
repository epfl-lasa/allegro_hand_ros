#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import math
import numpy as np

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
        self.RGB = [0, 0, 0]

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

    def _set_colour(self, R=1.0, G=0.0, B=0.0):
        self.marker_object.color.r = R
        self.marker_object.color.g = G
        self.marker_object.color.b = B
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

    def _set_scale(self,
                   diameter=0.01,
                   head_diameter=0.02,
                   head_length=0.0):
        self.marker_object.scale.x = diameter
        self.marker_object.scale.y = head_diameter
        self.marker_object.scale.z = head_length

    def update_marker(self, p1, p2):
        if not self.updated:
            self.marker_object.header.stamp = rospy.get_rostime()
            self._set_points([p1, p2])
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
        if n > 0:
            force_scale = 0.5
            Fx = data.states[0].total_wrench.force.x * force_scale
            Fy = data.states[0].total_wrench.force.y * force_scale
            Fz = data.states[0].total_wrench.force.z * force_scale

            Px = data.states[0].contact_positions[0].x
            Py = data.states[0].contact_positions[0].y
            Pz = data.states[0].contact_positions[0].z

            self.markers[ii].update_marker([Px, Py, Pz], [Px+Fx, Py+Fy, Pz+Fz])
        else:
            self.markers[ii].update_marker([0, 0, 0], [0, 0, 0])

if __name__ == '__main__':
    rospy.init_node('rviz_gazebo_contacts', anonymous=True)
    rate = rospy.Rate(30)
    
    ns = rospy.get_namespace()
    # ns = "/allegro_hand_right/"
    contact_display = ContactsDisplay(ns, links)

    while not rospy.is_shutdown():
        contact_display.publish()
        rate.sleep()
