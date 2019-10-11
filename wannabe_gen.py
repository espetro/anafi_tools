#!/usr/bin/python
# -*- coding: utf-8 -*-
# How the simulation generation should go

from __future__ import print_function
from utils.utils import RunTask, BackgroundTask, print_start, print_error, setupRun, get_random_height_cmd
from generators.world_builder import WorldBuilder
from utils.configs import A_CONFIG

import os
import sys

sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/aggregate")
sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/telemetry")

# from master2 import MasterNode
from telemetry import TelemetryConsumer


# ==== FUNCTIONS =======
# ====================

def simulate(config):
    """Runs a simulation given the configuration dictionary"""    

    SPHINX_ROOT = os.getenv("SPHINX_ROOT")
    ACTOR = SPHINX_ROOT + "/actors/pedestrian.actor::name={}::path={}"
    GZ_DRONE = SPHINX_ROOT + "/drones/local_bebop2.drone"

    # ROScore doesnt need to be restarted each run
    roscore = BackgroundTask("roscore", log=False, wait=5)
    MOVE_UP_DRONE = "rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist {}"
    MOVE_RNG = 5  # moves the drone in the Z axis by 0.5 or -0.5


    for i in range(config["reps"]):
        print("\nRun no {}. Initial wait: {}".format(i, config["delay_start"]))

        # ==== PER-RUN SETUP ====        
        # =======================

        # Generate world files
        world = WorldBuilder(config)
        objects = world.get_object_models()
        (world_fpath, subj_fpath, peds_fpath) = world.get_paths()

        # nm[-1] gives the pedestrian number; nm ~= P1, P0, ..
        GZ_SUBJECT = ACTOR.format("subject", subj_fpath)
        GZ_PEDS = " ".join([ACTOR.format(nm[-1], txt) for (nm, txt) in peds_fpath])

        RANDOM_HEIGHT = get_random_height_cmd(config["set_height"])

        # ==== TASK DISPATCH ====
        # =======================

        sphinx = BackgroundTask(
            "sphinx {} {} {} {}".format(world_fpath, GZ_DRONE, GZ_SUBJECT, GZ_PEDS),
            log=False, wait=5
        )

        bebop_driver = BackgroundTask(
            "roslaunch bebop_driver bebop_node.launch",
            log=False, wait=10
        )
        
        RunTask("rostopic pub --once /bebop/takeoff std_msgs/Empty", wait=5)

        if RANDOM_HEIGHT != "":
            for i in range(MOVE_RNG):
                RunTask(MOVE_UP_DRONE.format(RANDOM_HEIGHT), wait=4)

        teleop = BackgroundTask(
            "terminator -e 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/bebop/cmd_vel'",
            shell=True, wait=0.5
        )

        # Run while teleop is active
        data_logger = MasterNode(CONFIG, teleop)
        data_logger.start()

        # ==== WAIT UNTIL EXPERT POLICY RECORDING IS FINISHED ====
        # ========================================================
        teleop.wait()

        data_logger.stop()  # saves-closes the .CSV when finished
        bebop_driver.kill()
        sphinx.kill()

    roscore.kill()  # kills roscore after ALL runs


# ==== MAIN FLOW ====
# ===================

if __name__ == "__main__":
    simulate(A_CONFIG)
