from __future__ import print_function, absolute_import

class DroneModel:
    def __init__(self, num_s_samples,
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

    def reset(self):
        """Resets the model to default values"""
        self.pos = [(-1, None), (-1, None), (-1, None)]
        self.vel = [(-1, None), (-1, None), (-1, None)]
        self.acc = [(-1, None), (-1, None), (-1, None)]

    def set_pos_val(self, ts, uid, val):
        if uid == "x":
            self.pos[0] = (ts, val)
        elif uid == "y":
            self.pos[1] = (ts, val)
        elif uid == "z":
            self.pos[2] = (ts, val)

    def check_if_pos(self, uid):
        val = {"x": self.pos[0][1], "y": self.pos[1][1], "z": self.pos[2][1]}.get(uid)
        return val == None

    def set_vel_val(self, ts, uid, val):
        if uid == "x":
            self.vel[0] = (ts, val)
        elif uid == "y":
            self.vel[1] = (ts, val)
        elif uid == "z":
            self.vel[2] = (ts, val)

    def check_if_vel(self, uid):
        val = {"x": self.vel[0][1], "y": self.vel[1][1], "z": self.vel[2][1]}.get(uid)
        return val == None

    def set_acc_val(self, ts, uid, val):
        if uid == "x":
            self.acc[0] = (ts, val)
        elif uid == "y":
            self.acc[1] = (ts, val)
        elif uid == "z":
            self.acc[2] = (ts, val)

    def check_if_acc(self, uid):
        val = {"x": self.acc[0][1], "y": self.acc[1][1], "z": self.acc[2][1]}.get(uid)
        return val == None

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
        return ((-1) != t1 == t2 == t3) and all([t is not None for t in [x,y,z]])

    def complete(self):
        """
        Checks if the tuple data is right, i.e., all the timestamps are
        the same and the data is not empty.
        """
        return DroneModel._tuple_complete(self.pos) and \
            DroneModel._tuple_complete(self.vel) and \
            DroneModel._tuple_complete(self.acc)
    