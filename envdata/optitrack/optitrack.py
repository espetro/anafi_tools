#!/usr/bin/env python3.5

# === Instructions to bridge ROS1 topics to ROS2 topics ===
# (Or playback with rosbag)
# 1. Run roscore and the node or rosbag (with loop, -l)
# 2. Run 'ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics'.
#    a warning message will show up (dismatch between /clock and /Time).
#    This warning only happens when playing a rosbag.
# 3. Check 'ros2 topic list'

# before running this script, source both ROS2 and Olympe
# run "load_olympe" and once inside, "load_ros2"

import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# PoseStamped is composed by:
# msg.Header, msg.Pose(msg.Point(x,y,z)) 'position',
# msg.Quaternion(x,y,z,w) 'orientation' 

class OptiTrackConsumer(Node):

    def __init__(self):
        """"""
        super().__init__("listener")
        self.pose_drone = self.create_subscription(
            PoseStamped,
            "vrpn_client_node/drone/pose",
            self.drone_cb
        )

        self.pose_subj = self.create_subscription(
            PoseStamped,
            "vrpn_client_node/subject/pose",
            self.subj_cb
        )        

    def __str__(self):
        """"""
        pass

    @staticmethod
    def _Point2list(point):
        """
        Transforms a ROS2 geometry_msgs.msg.Point in a Python list
        :param point: A geometry_msgs.msg.Point object
        :returns: a Python list
        """
        return [point.x, point.y, point.z]

    @staticmethod
    def _Quaternion2list(quat):
        """
        Transforms a ROS2 geometry_msgs.msg.Quaternion in a Python list
        :param point: A geometry_msgs.msg.Quaternion object
        :returns: a Python list
        """
        return [quat.x, quat.y, quat.z, quat.w]

    @staticmethod
    def _Header2dict(header):
        """
        Transforms a ROS2 geometry_msgs.msg.Quaternion in a Python list
        :param point: A geometry_msgs.msg.Quaternion object
        :returns: a Python list
        """
        return {
            "time_sec": header.stamp.sec,
            "time_nano": header.stamp.nanosec,
            "frame_id": header.frame_id
        }

    def drone_cb(self, msg):
        """"""
        inf = OptiTrackConsumer._Header2dict(msg.header)
        pos = OptiTrackConsumer._Point2list(msg.pose.position)
        ori = OptiTrackConsumer._Quaternion2list(msg.pose.orientation)
        self.get_logger().info(
            "-----\nI heard {}\n{}\n{}\n-----".format(
                inf, pos, ori
            )
        )

    def subj_cb(self, msg):
        """"""
        pass

# if __name__ == "__main__":
#     # x = PoseStamped()
#     # print(x)
#     # print(dir(x))

#     rclpy.init(args=None)

#     node = OptiTrackPose()
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()