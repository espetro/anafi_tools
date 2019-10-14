
from __future__ import print_function, absolute_import
from models.cube import CubeShape

class TopDoorRailShape(CubeShape):
    model = "model://top_door_rail"

    def __init__(self, oid, pose, orient, dims=["0.25","2.0","0.1"]):
        """
        :param oid: Object ID
        :param pose: A tuple (x,y,z) being the centered centroid of the box
        :param orient: A tuple (pitch, yaw, roll) being the box orientation
        :param dims: A tuple (dim_x, dim_y, dim_z) of the box geometry. By default
                     it is _____
        """
        super(TopDoorRailShape, self).__init__(oid, pose, orient, dims)
        self.name = "TopDoorRail"
        