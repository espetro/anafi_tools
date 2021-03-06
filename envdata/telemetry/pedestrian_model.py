from __future__ import print_function, absolute_import
from numpy.linalg import norm as euclidean
from numpy import array

class PedestrianModel(object):

    def __init__(self, num_s_samples,
        pose_x=None, pose_y=None, pose_z=None, rad=0.6, height=1.7):
        """
        Height is defined in $SPHINX_ROOT/actors/pedestrian.actor
        Radius is checked by placing the pedestrian in a cylinder in the simulation.
        rad=0.5 also works (still ok)

        :param pose_x:
        :param pose_y:
        :param pose_z:
        :param rad:
        :param height:
        """
        self.x = (-1, pose_x)
        self.y = (-1, pose_y)
        self.z = (-1, pose_z)
        self.radius = float(rad)
        self.height = float(height)

    def __repr__(self):
        return "(Pedestrian ({},{},{}))".format(self.x[1], self.y[1], self.z[1])

    def reset(self):
        """Resets the model to default values"""
        self.x = (-1, None)
        self.y = (-1, None)
        self.z = (-1, None)
        
    def set_val(self, ts, uid, val):
        if uid == "x":
            self.x = (ts, val)
        elif uid == "y":
            self.y = (ts, val)
        elif uid == "z":
            self.z = (ts, val)

    def check_if_pos(self, uid):
        val = {"x": self.x[1], "y": self.y[1], "z": self.z[1]}.get(uid)
        return val == None

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
        return (t1 == t2 and t2 == t3) and all([t is not None for t in [x,y,z]])

    def distance_to(self, point):
        """
        :param point: A tuple (x,y,z)
        """
        curr_pose = array([self.x[1], self.y[1], self.z[1]])

        if point[2] <= self.height:
            # Ignore Z axis
            dist = euclidean(array(point[:2]), curr_pose[:2]) - self.radius
        else:
            # Use maximum height and no radius
            centr = curr_pose + array([0, 0, self.height])
            dist = euclidean(array(centr), array(point))
        return dist
