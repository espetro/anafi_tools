
from numpy.linalg import norm as euclidean
from numpy import array

class SphereShape(object):

    def __init__(self, pose, rad):
        self.pose = pose
        self.radius = float(rad)

    def distance_to(self, point):
        """
        :param point: A tuple (x,y,z)
        """
        return euclidean(array(self.pose[:3]), array(point)) - self.radius
