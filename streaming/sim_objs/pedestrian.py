
from numpy.linalg import norm as euclidean
from numpy import array

class PedestrianModel(object):

    def __init__(self, pose_x=0.0, pose_y=0.0, pose_z=0.0, rad=0.6, height=1.7):
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
        self.x, self.y, self.z = pose_x, pose_y, pose_z
        self.radius = float(rad)
        self.height = float(height)

    def set_pose(self, new_pose):
        """
        :param new_pose:
        """
        self.pose = new_pose

    def distance_to(self, point):
        """
        :param point: A tuple (x,y,z)
        """
        if point[2] <= self.height:
            # Ignore Z axis
            dist = euclidean(array(self.pose[:2]), array(point[:2])) - self.radius
        else:
            # Use maximum height and no radius
            centr = array(self.pose) + array([0, 0, self.height])
            dist = euclidean(array(centr), array(point))
        return dist
