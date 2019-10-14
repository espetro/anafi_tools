from __future__ import print_function, absolute_import
from numpy.linalg import norm as euclidean
from numpy import array

class PedestrianModel(object):

    def __init__(self, rad=0.6, height=1.7):
        """
        Setups a pedestrian object model linked to the Gazebo model.

        :param pose_x:
        :param pose_y:
        :param pose_z:
        :param rad: Radius is checked by placing the pedestrian in a cylinder in the simulation.
        :param height: Height is defined in $SPHINX_ROOT/actors/pedestrian.actor
        """
        self.ts = -1
        self.radius = float(rad)
        self.height = float(height)
        self.reset()

    def __repr__(self):
        return "(Pedestrian ({},{},{}))".format(*self.get_pos())

    def reset(self):
        self.pos = {"x": (-1, None), "y": (-1, None), "z": (-1, None)}
        
    def set_val(self, ts, uid, val):
        self.ts = ts
        self.pos.update({uid: (ts, val)})

    def get_pos(self):
        """A nice way to implement it would be checking timestamp on 3 coords"""
        return [val for (_, val) in self.pos.values()]

    def complete(self):
        """
        Checks if the position data is right, i.e., all the timestamps are
        the same and the data is not empty.
        """
        (t1, x), (t2, y), (t3, z) = self.pos.values()
        return (t1 == t2 and t2 == t3) and all([t is not None for t in [x,y,z]])

    def distance_to(self, point):
        """
        :param point: A tuple (x,y,z)
        """
        curr_pose = array(self.get_pos())

        if point[2] <= self.height:
            # Ignore Z axis
            dist = euclidean(array(point[:2]), curr_pose[:2]) - self.radius
        else:
            # Use maximum height and no radius
            centr = curr_pose + array([0, 0, self.height])
            dist = euclidean(array(centr), array(point))
        return dist
