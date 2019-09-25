
from numpy.linalg import norm as euclidean
from numpy import array

class PedestrianShape(object):

    def __init__(self, orig_pose=[0,0,0,0,0,0], rad=4, height=2.2):
        self.pose = orig_pose
        self.radius = float(rad)
        self.height = float(height)

    def set_pose(self, new_pose):
        self.pose = new_pose

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
