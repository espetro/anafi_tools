from __future__ import print_function, absolute_import

class DroneModel:
    def __init__(self,
        pose_x=None, pose_y=None, pose_z=None,
        vel_x=None, vel_y=None, vel_z=None,
        acc_x=None, acc_y=None, acc_z=None):

        self.pos = [(-1, pose_x), (-1, pose_y), (-1, pose_z)]
        self.vel = [(-1, vel_x), (-1, vel_y), (-1, vel_z)]
        self.acc = [(-1, acc_x), (-1, acc_y), (-1, acc_z)]

    def __repr__(self):
        return "(Drone in {} at speed {}, {})".format(
            self.get_pos(), self.get_vel(), self.get_acc()
        )

    def set_pos_val(self, ts, uid, val):
        if uid == "x":
            self.pos[0] = (ts, val)
        elif uid == "y":
            self.pos[1] = (ts, val)
        elif uid == "z":
            self.pos[2] = (ts, val)

    def set_vel_val(self, ts, uid, val):
        if uid == "x":
            self.vel[0] = (ts, val)
        elif uid == "y":
            self.vel[1] = (ts, val)
        elif uid == "z":
            self.vel[2] = (ts, val)

    def set_acc_val(self, ts, uid, val):
        if uid == "x":
            self.acc[0] = (ts, val)
        elif uid == "y":
            self.acc[1] = (ts, val)
        elif uid == "z":
            self.acc[2] = (ts, val)

    def get_pos(self):
        """A nice way to implement it would be checking timestamp on 3 coords"""
        # cannot be reassigned within a tuple
        return tuple([x for (_,x) in self.pos])

    def get_acc(self):
        return tuple([a for (_,a) in self.acc])

    def get_vel(self):
        return tuple([v for (_,v) in self.vel])

    def get_data(self):
        return (self.get_pos(), self.get_vel(), self.get_acc())

    @staticmethod
    def _tuple_complete(three):
        """
        Checks if the tuple data is right, i.e., all the timestamps are
        the same and the data is not empty.
        """
        (t1, x), (t2, y), (t3, z) = three
        return (t1 == t2 == t3) and all([t is not None for t in [x,y,z]])

    def complete(self):
        """
        Checks if the tuple data is right, i.e., all the timestamps are
        the same and the data is not empty.
        """
        return DroneModel._tuple_complete(self.pos) and \
            DroneModel._tuple_complete(self.vel) and \
            DroneModel._tuple_complete(self.acc)
    