
# from streaming.sim_objs.cube import CubeShape
from cube import CubeShape

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
