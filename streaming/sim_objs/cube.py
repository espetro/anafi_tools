
from numpy.linalg import norm as euclidean
from numpy import array
from math import sin, cos, sqrt

class CubeShape(object):

    def __init__(self, oid, pose, orient, dims):
        """
        :param oid: Object ID
        :param pose: A tuple (x,y,z) being the centered centroid of the box
        :param orient: A tuple (pitch, yaw, roll) being the box orientation
        :param dims: A tuple (dim_x, dim_y, dim_z), being the diameter of the box
                     in each axis
        """
        self.nid = oid
        self.pose = pose
        self.orient = orient
        self.dims = [float(x) for x in dims]
        self.rads = [d / 2 for d in self.dims]

    def _align_to_orig(self):
        """
        """
        return self.pose

    @staticmethod
    def _euclidean(poseA, poseB):
        """
        :param poseA:
        :param poseB:
        """
        # return euclidean(array(poseA), array(poseB))
        return sqrt(sum([(a - b)**2 for a,b in zip(poseA, poseB)]))

    
    def _get_corner_dist(self, points):
        """
        Computes the distance from the box to a point by obtaining the closest
        distance from the point to one of the 30 corner/face points.
        (5 points per face: corners + center)

        :param points:
        """

        # pose_aligned = self.align_to_orig()

        # # X-centered faces
        # r_main, r_other = rads[0], rads[1:]
        # for x in range(2):
        #     x = array(center_pose) + array()
        
        # self.rads[2]

        # self.rads[0], self.rads[1]
        # self.rads[0], -self.rads[1]
        # -self.rads[0], self.rads[1]
        # -self.rads[0], -self.rads[1]

        # Y-centered faces

        # Z-centered faces

        # fcx1: face center x1
        # fcx1 = array(pose_aligned) - array([self.rads[0], 0, 0])
        # fcx2 = array(pose_aligned) + array([self.rads[0], 0, 0])

        # fcy1 = array(pose_aligned) - array([0, self.rads[1], 0])
        # fcy2 = array(pose_aligned) - array([0, self.rads[1], 0])

        # fcz1 = array(pose_aligned) - array([0, 0, self.rads[2]])
        # fcz2 = array(pose_aligned) - array([0, 0, self.rads[2]])

        # d = 1e6
        # f_final = None
        # for f in [fcx1, fcx2, fcy1, fcy2, fcz1, fcz2]:
        #     f_dist = CubeShape._euclidean(point, f)
        #     if d > f_dist:
        #         d = f_dist
        #         f_final = f

        # # Get 4 corners
        # corner_tl = f_final + array()

    def _get_transform_dist(self, point):
        """
        Computes the distance between the box and a 3D point by using coordinate
        transforms. Note that the drone is pitch-yaw-invariant in order to get
        stable.

        :param point: A tuple (x,y,z)
        """

        # Pitch and Roll become invariant (in order to stabilize the drone)
        _, yaw, _ = self.orient

        # (1) Get point in the wall reference
        pt_to_rotate = array(point) - array(self.pose)
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
        pt_unrotated += array(self.pose)

        return 0, pt_unrotated, point, self.pose
        # return CubeShape._euclidean(point, pt_unrotated)

    def distance_to(self, point):
        """
        Computes the distance from a point in the 3D space to the closest shape
        point in the 3D box.

        :param point: A tuple (x,y,z)
        """

        # dist = self._get_corner_dist(point)
        dist = self._get_transform_dist(point)

        return dist

        