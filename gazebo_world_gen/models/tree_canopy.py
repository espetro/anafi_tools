from __future__ import print_function, absolute_import
from numpy.linalg import norm as euclidean
from numpy import array

class TreeCanopyShape(object):
    model = "model://tree_canopy"

    def __init__(self, oid, pose, rad="1.0"):
        self.name = "TreeCanopy"
        self.oid = oid
        self.pose = [float(p) for p in pose]
        self.radius = float(rad)

    def __str__(self):
        return "TreeCanopy object with ID {} at {} with radius: {}".format(
            self.oid,
            self.pose,
            self.radius
        )

    def distance_to(self, point):
        """
        :param point: A tuple (x,y,z)
        """
        return euclidean(array(self.pose[:3]), array(point)) - self.radius
