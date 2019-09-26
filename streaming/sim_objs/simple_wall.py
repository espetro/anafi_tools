
from streaming.sim_objs.cube import CubeShape

class SimpleWallShape(CubeShape):

    def __init__(self, oid, pose, orient, dims=["0.2", "5.0", "2.5"]):
        """
        :param oid: Object ID
        :param pose: A tuple (x,y,z) being the centered centroid of the box
        :param orient: A tuple (pitch, yaw, roll) being the box orientation
        :param dims: A tuple (dim_x, dim_y, dim_z) of the box geometry. By default
                     it is _____
        """
        self.name = "simple_wall"
        super().__init__(oid, pose, orient, dims)

    def distance_to(self, point):
        pass
        
