#!/usr/bin/env python
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

import numpy as np
import matplotlib.pyplot as plt

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ContactsState


# ----- Global Variables -----
LINKS = [
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
    """Basic marker class to visualize on Rviz"""

    def __init__(self, frame, ns, unique_id):
        """Constructor of the Basic Marker Class

        Args:
            frame (str): link to which marker is attached
            ns (str): namespace of the marker
            unique_id (int): unique id for the marker
        """
        self._cmap = plt.cm.get_cmap('autumn_r')
        self._norm = plt.Normalize(vmin=0.0, vmax=5.0, clip=True)

        self.marker_object = Marker()
        self.marker_object.type = Marker.ARROW
        self.marker_object.action = Marker.ADD
        self.marker_object.ns = ns
        self.marker_object.id = unique_id
        self.marker_object.header.frame_id = "world"    # [frame] - Gazibo sends values in global frame
        self.marker_object.frame_locked = True
        self._set_scale()
        self._set_colour()
        self._set_points([[0, 0, 0], [0, 0, 0]])
        self.updated = True

        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0.1)

    def _set_points(self, points):
        """Set local position of the marker

        Args:
            points (list[list[float]]): [description]
        """
        self.marker_object.points = []
        for [x, y, z] in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            self.marker_object.points.append(point)

    def _set_colour(self, pressure=0.0):
        """Set marker color based on the pressure

        Args:
            pressure (float, optional): Pressure/force on the link. Defaults to 0.0.
        """
        rgba = self._cmap(self._norm(pressure))
        self.marker_object.color.r = rgba[0]
        self.marker_object.color.g = rgba[1]
        self.marker_object.color.b = rgba[2]
        self.marker_object.color.a = rgba[3]

    def _set_scale(self,
                   diameter=0.005,
                   head_diameter=0.008,
                   head_length=0.0):
        """Sets marker scale

        Args:
            diameter (float, optional): Diameter of the arrow. Defaults to 0.005.
            head_diameter (float, optional): Diameter of the arrow head. Defaults to 0.008.
            head_length (float, optional): Length of the arrow head. Defaults to 0.0.
        """
        self.marker_object.scale.x = diameter
        self.marker_object.scale.y = head_diameter
        self.marker_object.scale.z = head_length

    def update_marker(self, p1, p2, pressure):
        """Update marker properties

        Args:
            p1 (list[float]): Start point of the marker arrow
            p2 (list[float]): End point of the marker arrow
            pressure (float): Pressure on the link
        """
        if not self.updated:
            self.marker_object.header.stamp = rospy.get_rostime()
            self._set_points([p1, p2])
            self._set_colour(np.abs(pressure))
            self.updated = True

    def get_marker(self):
        """Return `Marker` object for ROS message

        Returns:
            `Marker`: Marker for publishing to ROS network
        """
        self.updated = False
        return self.marker_object


class ContactsDisplay():
    """Publish `MarkerArray` for Rviz visulization of gazebo contact sensor"""

    def __init__(self, ns, links):
        """Constructor for ContactsDisplay class

        Args:
            ns (str): namespace of the marker array
            links (list[str]): List of all links of the sensor
        """
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
                queue_size=1,
            )
            for i in range(len(links))
        ]
        self.publisher = rospy.Publisher(
            "rviz_gazebo_contact",
            MarkerArray,
            queue_size=1,
        )

    def publish(self):
        """Publishes `MarkerArray` to the ROS network"""
        markerarray_object = MarkerArray()
        markerarray_object.markers = [
            marker.get_marker()
            for marker in self.markers
        ]
        self.publisher.publish(markerarray_object)

    def contact_callback(self, data, ii):
        """Analyse the gazebo data and update the markers for the link

        Args:
            data (ContactsState): `ContactsState` message from the gazebo
            ii (int): link identifier
        """
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

            if total_normal_force == 0:
                P = np.zeros((3, 1))
                N = np.zeros((3, 1))
            else:
                P = weighted_P / total_normal_force
                N = weighted_N / total_normal_force

            F = np.asarray([
                data.states[0].total_wrench.force.x,
                data.states[0].total_wrench.force.y,
                data.states[0].total_wrench.force.z,
            ])
            pressure = total_normal_force   # or np.dot(F, N)

            temp = P + np.sign(pressure) * N * 0.05
            self.markers[ii].update_marker(P, temp, pressure)


if __name__ == '__main__':
    rospy.init_node('rviz_gazebo_contacts', anonymous=True)
    rate = rospy.Rate(30)

    ns = rospy.get_namespace()
    contact_display = ContactsDisplay(ns, LINKS)

    while not rospy.is_shutdown():
        contact_display.publish()
        rate.sleep()
