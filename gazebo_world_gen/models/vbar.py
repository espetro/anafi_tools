
from __future__ import print_function, absolute_import
from numpy.linalg import norm as euclidean
from numpy import array

class VerticalBarShape(object):
    model = "model://vertical_bar"

    def __init__(self, oid, pose, rad="0.1", height="2.0"):
        """
        :param oid: Object ID
        """
        self.name = "VerticalBar"
        self.oid = oid
        self.pose = pose
        self.radius = float(rad)
        self.height = float(height)

    def __str__(self):
        return "VerticalBar object with ID {} at {} with radius {}, height {}".format(
            self.oid,
            self.pose,
            self.radius,
            self.height
        )

    def distance_to(self, point):
        """
        :param point: A tuple (x,y,z)
        """
        if point[2] <= self.height:
            # Ignore Z axis
            dist = euclidean(array(self.pose[:2]), array(point[:2])) - self.radius
        else:
            # Use maximum height and no radius
            centr = array(self.pose) + array([0, 0, self.height])
            dist = euclidean(array(centr), array(point))
        return dist
