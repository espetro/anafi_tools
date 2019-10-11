
# from streaming.sim_objs.cube import CubeShape
from cube import CubeShape
from numpy.linalg import norm as euclidean
from numpy import array
from math import sin, cos, sqrt

class WallBoxShape(CubeShape):
    model = "model://wall_box"

    def __init__(self, oid, pose, orient=[0.0, 0.0, 0.0], dims=["0.2", "5.0", "2.5"]):
        """
        :param oid: Object ID, usually the unique name used in the .world file
        :param pose: A tuple (x,y,z) being the centered centroid of the box
        :param orient: A tuple (pitch, yaw, roll) being the box orientation
        :param dims: A tuple (dim_x, dim_y, dim_z) of the box geometry. By default
                     it is _____
        """
        super(WallBoxShape, self).__init__(oid, pose, orient, dims)
        self.name = "WallBox"
        self.height = 3.75
        self.radius = 0.35

    def distance_to(self, point):
        """
        :param point: A tuple (x,y,z)
        """
        if point[2] <= self.height:
            # Ignore Z axis
            dist = CubeShape._euclidean(array(point[:2]), self.pose[:2]) - self.radius
        else:
            # Use maximum height and no radius
            centr = array(self.pose) + array([0, 0, self.height])
            dist = CubeShape._euclidean(array(centr), array(point))
        return dist
