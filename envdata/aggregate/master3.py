# -*- coding: utf-8 -*-

import sys
# load_olympe
# load_ros2
sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/optitrack")
sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/telemetry")
sys.path.append("/home/pachacho/Documents/anafi_tools/control_parrot_drones/drone")

from aggregator import AggregatorNode
from optitrack.optitrack import OptiTrackConsumer
from telemetry.telemetry import TelemetryConsumer
from drone.anafi import AnafiControl

class MasterNode:
    """
    Setups a node to take all the steps needed in handling the data, i.e.
    (1) Receive the data from streams (optitrack, telemetry)
    (2) Pre-process the data and compute both distances and forces
    (3) Obtain velocity commands (model-engine)
    (4) Set velocity commands in the drone (anafi)
    """
    def __init__(self, config):
        """
        :param config: A dictionary
        """
        self.agg = AggregatorNode()

        if config["simulated"] is True:
            self.stream = TelemetryConsumer()
            self.ip = AnafiControl.SIMULATED_IP
            self.stream.on_sample(self.on_telemetry_sample)
            self.bag = DataBag(
                simulated = True,
                config["objects"]
            )
        else:
            self.stream = OptiTrackConsumer()
            self.ip = AnafiControl.PHYSICAL_IP
            self.stream.on_sample(self.on_optitrack_sample)

        self.drone = AnafiControl(self.ip)
        self.drone.takeoff()

        self.stream.start()


    def on_telemetry_sample(self, msg):
        """
        Pre-process the data and, if the bag is full, log it and take actions.
        :param msg: A tuple ()
        """
        # Check if it's drone's 

    def on_optitrack_sample(self, msg):
        """
        Pre-process the data and, if the bag is full, log it and take actions.
        :param msg: A list of lists [x,y,z,0,0,w]
        """

        # An array [x,y,z,0,0,w]
        # agg.add_data(msg)
        # if agg.is_data_full():
            # distances, forces = agg.aggregate()
            # vel_cmd = model.fit(forces)
            # self.drone.set_vel(vel_cmd)
        return None
