
from numpy.linalg import norm as euclidean
from numpy import array

class CylinderShape(object):

    def __init__(self, pose, rad, height):
        self.pose = pose
        self.radius = float(rad)
        self.height = float(height)

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
