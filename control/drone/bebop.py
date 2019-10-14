# # -*- coding: utf-8 -*-
# Uncomment this if you want to use ROS1 with Bebop2.
# Take into account that pypi will stop working as rospy enforces Python 2.7

# from __future__ import absolute_import, print_function
# from geometry_msgs.msg import Twist
# from rospy import Subscriber, Publisher

# import rospy

# class BebopControl:
#     def __init__(self, cmd_vel_topic="/bebop/cmd_vel"):
#         """"""
#         rospy.init_node("bebop_control", anonymous=True)
#         self.cmd = rospy.Publisher(
#             cmd_vel_topic,
#             Twist,
#             queue_size=2
#         )
# 
#     def set_vel(self, twist):
#         """
#         Sets the drone velocity.
#         :param twist: An array [x,y,z,0,0,w]
#         """
#         msg = Twist()
#         msg.linear.x = twist[0]
#         msg.linear.y = twist[1]
#         msg.linear.z = twist[2]
#         msg.angular.z = twist[-1]
#         self.cmd.publish(msg)