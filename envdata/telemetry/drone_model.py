# -*- coding: utf-8 -*-
from __future__ import print_function, absolute_import

class DroneModel:
    def __init__(self):
        self.ts = -1
        self.reset()

    def __repr__(self):
        return "(Drone in {} at speed {}, {})".format(
            self.get_pos(), self.get_vel(), self.get_acc()
        )

    def reset(self):
        self.pos = {"x": (-1, None), "y": (-1, None), "z": (-1, None)}
        self.vel = {"x": (-1, None), "y": (-1, None), "z": (-1, None)}
        self.acc = {"x": (-1, None), "y": (-1, None), "z": (-1, None)}

    def set_pos_val(self, ts, uid, val):
        self.ts = ts
        self.pos.update({uid: (ts, val)})

    def set_vel_val(self, ts, uid, val):
        self.ts = ts
        self.vel.update({uid: (ts, val)})

    def set_acc_val(self, ts, uid, val):
        self.ts = ts
        self.acc.update({uid: (ts, val)})

    def get_pos(self):
        return [val for (_, (_, val)) in self.pos.items()]

    def get_vel(self):
        return [val for (_, (_, val)) in self.vel.items()]

    def get_acc(self):
        return [val for (_, (_, val)) in self.acc.items()]

    def get_data(self):
        return (self.get_pos(), self.get_vel(), self.get_acc())

    @staticmethod
    def _is_complete(vdict):
        """
        Checks if the dict data is right, i.e., all the timestamps are
        the same and the data is not empty.
        """
        (t1, x), (t2, y), (t3, z) = [tpl for (_, tpl) in vdict.items()]
        return (t1 == t2 == t3) and all([t is not None for t in [x,y,z]])

    def complete(self):
        """
        Checks if the tuple data is right, i.e., all the timestamps are
        the same and the data is not empty.
        """
        return DroneModel._is_complete(self.pos) and \
            DroneModel._is_complete(self.vel) and \
            DroneModel._is_complete(self.acc)
    