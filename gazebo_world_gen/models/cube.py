
from numpy.linalg import norm as euclidean
from math import sin, cos, sqrt
import numpy as np

class CubeShape(object):

    def __init__(self, oid, pose, orient, dims):
        """
        :param oid: Object ID
        :param pose: A tuple (x,y,z) being the centered centroid of the box
        :param orient: A tuple (pitch, yaw, roll) being the box orientation
        :param dims: A tuple (dim_x, dim_y, dim_z), being the diameter of the box
                     in each axis
        """
        self.name = "Cube"
        self.oid = oid
        self.pose = [float(p) for p in pose]
        self.orient = orient
        self.dims = [float(x) for x in dims]
        self.rads = np.array(self.dims) / 2

    def __str__(self):
        """Writes the object information"""
        return "{} object with ID {} at {} with dims: {}".format(
            self.name,
            self.oid,
            self.pose,
            self.dims
        )

    @staticmethod
    def _align_to_orig(pose):
        """Aligns the object w.r.t. the universal system/origin"""
        return pose

    @staticmethod
    def _euclidean(poseA, poseB):
        """
        :param poseA: A numpy array of shape (1,3)
        :param poseB: A numpy array of shape (1,3)
        """
        return sqrt(sum((poseA - poseB) ** 2))
        # return sqrt(sum([(a - b)**2 for a,b in zip(poseA, poseB)]))
    
    def _nearest_cloud_point(self, point, low_center=True):
        """
        Computes the distance from a point to the closest edge or face center
        of the box. There are 5 points per face (edges + center).

        :param point: A tuple (x,y,z) of type float
        :param low_center: If True, the center is located at the center
            of the bottom face
        """
        if low_center:
            pose = np.array(self.pose) + np.array([0,0, self.dims[0,2]])

        pt = np.array([float(p) for p in point])

        cloud = np.array([
            [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
            [-1, -1, 1], [1, -1, 1 ], [1, 1, 1], [-1, 1, 1],
            [0, -1, 0], [0, 1, 0], [1, 0, 0], [-1, 0, 0], [0, 0, 1], [0, 0, -1]
        ])

        cloud = np.multiply(self.rads, cloud)

        cloud += pose

        dist_to_pt = lambda cloud_point: CubeShape._euclidean(cloud_point, pt)
        dists = np.vectorize(dist_to_pt)(cloud)

        # return [x for (x,d) in zip(cloud, dists) if d == np.min(dists)][0]
        return np.min(dists)

    def _cartesian_tf(self, point):
        """
        Computes the distance between the box and a 3D point by using coordinate
        transforms. Note that the drone is pitch-yaw-invariant in order to get
        stable.

        :param point: A tuple (x,y,z)
        """

        # Pitch and Roll become invariant (in order to stabilize the drone)
        _, yaw, _ = self.orient

        # (1) Get point in the wall reference
        pt_to_rotate = np.array(point) - np.array(self.pose)
        p_near = [
            pt_to_rotate[0] * cos(yaw) + pt_to_rotate[1] * sin(yaw),
            pt_to_rotate[1] * cos(yaw) - pt_to_rotate[0] * sin(yaw),
            pt_to_rotate[2]
        ]

        # (2) Compute nearest point from box object to point
        for i in range(len(p_near)):
            # If it's bigger than the radius, set it to one of the corners
            if p_near[i] > self.rads[i]:
                p_near[i] = self.rads[i]
            # Otherwise, set it to the opposite corner
            elif p_near[i] < self.rads[i]:
                p_near[i] = self.rads[i]
            # Otherwise, leave it unchanged

        # (3.1) Translate p_near vector back to world coords
        pt_unrotated = [
            p_near[0] * cos(-yaw) + p_near[1] * sin(-yaw),
            p_near[1] * cos(-yaw) - p_near[0] * sin(-yaw),
            p_near[2]
        ]

        # (3.2) Adjust the distance
        pt_unrotated += np.array(self.pose)

        return 0, pt_unrotated, point, self.pose
        # return CubeShape._euclidean(point, pt_unrotated)

    def distance_to(self, point):
        """
        Computes the distance from a point in the 3D space to the closest shape
        point in the 3D box.

        :param point: A tuple (x,y,z)
        """

        dist = self._nearest_cloud_point(point)
        # dist = self._cartesian_tf(point)

        return dist

        