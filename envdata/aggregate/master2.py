# -*- coding: utf-8 -*-
from __future__ import absolute_import, print_function

import sys

sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/telemetry")

class MasterNode:
    """Logs all the telemetry data into a CSV"""
    def __init__(self):
        """
        Setups a node to take all the steps needed in handling the data, i.e.
        (1) Receive the data from streams (optitrack, telemetry)
        (2) Pre-process the data and compute both distances and forces
        (3) Obtain velocity commands (model-engine)
        (4) Set velocity commands in the drone (anafi)
        """

        self.agg = AggregatorNode()
        self.objs = objs  # A list of object models
        
        if simulated:
            self.stream = TelemetryConsumer(
                processing_fun=self.on_data,
                objs=self.objs
            )
            self.ip = AnafiControl.SIMULATED_IP
        else:
            self.stream = OptiTrackConsumer(processing_fun=self.on_data)
            self.ip = AnafiControl.PHYSICAL_IP

        self.drone = AnafiControl(self.ip)

    def start(self):
        self.stream.start()

    def stop(self):
        self.stream.stop()

    def on_data(self, data):
        """
        Digests all the data streamed from drones, pedestrians and objects
        :param data: A dict holding 5 keys: ts, drone, subject, peds, objs
            Each value is a list of models
        """
        # aggregate all data
        forces = self.compute_forces(data)
        # if model, then do model.fit
        if model is not None:
            cmd_vel = model.fit(forces)
            self.drone.piloting_cmd(*cmd_vel)
        # then log it in a .csv
        self.log2csv(data, forces, [...])
        
    def compute_forces(self, data):
        pass

    def compute_distances(self, p1, p2):
        pass