#!/usr/bin/env python
# Optitrack publishes data to ROS1 topics
# This source code should be moved to a ROS2 workspace

import rclpy
import olympe

from olympe.messages.Piloting import moveBy
from numpy.linalg import norm as euclidean
from numpy import array

# Optitrack markers constants (subject, pedestrians and drone)
MARKER_DRONE = 1
MARKER_SUBJECT = 2
MARKER_PEDS = [3,4]

#============== Optitrack Reflective Markers ==============
#==========================================================

class OptitrackMRB:
    def __init__(self):
        """"""
        pass

    def distance_to(self, point):
        """
        Computes the euclidean distance given two points.
        :param point: A tuple (x,y,z)
        """
        return euclidean(array(self.pose), array(point))

#============== Bridge for ROS1-ROS2 ======================
#==========================================================

# See more at
# https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst
ro
if __name__ == "__main__":
    # If ran as a script
    # rospy.init_node("optitrack_marker_consumer", anonymous=True)