# -*- coding: utf-8 -*-
from __future__ import print_function
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

import olympe

class AnafiControl:
    PHYSICAL_IP = "192.168.42.1"
    SIMULATED_IP = "10.202.0.1"

    def __init__(self, ip="10.202.0.1"):
        """
        Log level can be: debug= 4, info= 3, warning= 2, error= 1, critical= 0
        """
        self.drone = olympe.Drone(ip, loglevel=4)

        self.drone.connection()
        self.drone.start_piloting()

    def set_vel(self, twist):
        """
        Sets the drone velocity.
        :param twist: An array [x,y,z,0,0,w]
        """
        self.drone.piloting_pcmd(
            twist[0], twist[1], twist[2], twist[-1],
            piloting_time=1
        )

    def stop(self):
        self.drone.stop_piloting()
        self.drone.disconnection()

    def takeoff(self):
        self.drone(TakeOff()).wait()

    def land(self, timeout=3):
        self.drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=timeout)
        ).wait()