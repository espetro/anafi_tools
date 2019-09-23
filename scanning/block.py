
from numpy.linalg import norm as euclidean
from numpy import array

class BlockShape(object):

    def __init__(self, pose, dimensions):
        self.pose = pose
        self.dims = [float(x) for x in dimensions]

    def distance_to(self, point):
        """
        :param point: A tuple (x,y,z)
        """
        # Transport the system

        # Foreach dimension, compute the distance

        # Group the dimensions and transform the system to the 0,0,0
        
