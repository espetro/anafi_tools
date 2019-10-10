from __future__ import print_function, absolute_import

class DroneModel:
    def __init__(self, pose_x=None, pose_y=None, pose_z=None):
        self.x = (-1, pose_x)
        self.y = (-1, pose_y)
        self.z = (-1, pose_z)

    def __repr__(self):
        return "(Drone ({},{},{}))".format(self.x[1], self.y[1], self.z[1])

    def set_val(self, ts, uid, val):
        if uid == "x":
            self.x = (ts, val)
        elif uid == "y":
            self.y = (ts, val)
        elif uid == "z":
            self.z = (ts, val)

    def get_pos(self):
        """A nice way to implement it would be checking timestamp on 3 coords"""
        return (self.x[1], self.y[1], self.z[1])

    def complete(self):
        """
        Checks if the position data is right, i.e., all the timestamps are
        the same and the data is not empty.
        """
        (t1, x) = self.x
        (t2, y) = self.y
        (t3, z) = self.z
        return (t1 == t2 == t3) and all([t is not None for t in [x,y,z]])
    