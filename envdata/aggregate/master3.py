# -*- coding: utf-8 -*-

import sys

# load_olympe
# load_ros2
sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/optitrack")

class MasterNode:

    def __init__(self):
        """
        Setups a node to take all the steps needed in handling the data, i.e.
        (1) Receive the data from streams (optitrack, telemetry)
        (2) Pre-process the data and compute both distances and forces
        (3) Obtain velocity commands (model-engine)
        (4) Set velocity commands in the drone (anafi)
        """

        agg = AggregatorNode()

        if simulated:
            self.stream = TelemetryConsumer()
            self.ip = AnafiControl.SIMULATED_IP
        else:
            self.stream = OptiTrackConsumer()
            self.ip = AnafiControl.PHYSICAL_IP

        self.drone = AnafiControl(self.ip)
        self.stream.start()


    def on_sample(self):
        # An array [x,y,z,0,0,w]
        # agg.add_data(msg)
        # if agg.is_data_full():
            # distances, forces = agg.aggregate()
            # vel_cmd = model.fit(forces)
            # self.drone.set_vel(vel_cmd)
        return None
