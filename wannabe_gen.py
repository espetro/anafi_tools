#!/usr/bin/python
# -*- coding: utf-8 -*-
# How the simulation generation should go

from __future__ import print_function, absolute_import
from utils.utils import RunTask, BackgroundTask, print_start, print_error, setupRun, get_random_height_cmd
from generators.world_builder import WorldBuilder
from utils.configs import A_CONFIG
from random import random

import os
import sys
import shutil

sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/aggregate")
sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/telemetry")

# pip install -e gazebo... to get changes updated
# from master2 import MasterNode
from data_logger import DataLogger


# ==== FUNCTIONS =======
# ====================
from threading import Thread
from subprocess import call
from time import sleep

class IlloKeWapo:
    def __init__(self, cmd, wait=5):
        self.cmd = cmd
        sleep(wait)
        self.thread = Thread(target=self.start_cmd)

    def start_cmd(self):
        call(self.cmd, shell=True)


def simulate(config):
    """Runs a simulation given the configuration dictionary"""    

    SPHINX_ROOT = os.getenv("SPHINX_ROOT")
    ACTOR = SPHINX_ROOT + "/actors/pedestrian.actor::name={}::path={}"
    GZ_DRONE = SPHINX_ROOT + "/drones/local_bebop2.drone"

    MOVE_UP_DRONE = "rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist {}"
    MOVE_RNG = 5  # moves the drone in the Z axis by 0.5 or -0.5

    # ROScore doesnt need to be restarted each run
    roscore = BackgroundTask("roscore", log=False, wait=5)

    try:
        for i in range(config["reps"]):

            print("\nRun no {}. Initial wait: {}".format(i, config["delay_start"]))
            config["run_name"] = "run{}".format(i)

            # ==== PER-RUN SETUP ====        
            # =======================

            # Generate world files
            print("Generating new world.")

            world = WorldBuilder(config)
            objects = world.get_object_models()
            (world_fpath, subj_fpath, peds_fpath) = world.get_paths()
            config["final_peds"] = world.get_num_peds()

            # nm[-1] gives the pedestrian number; nm ~= P1, P0, ..
            GZ_SUBJECT = ACTOR.format("subject", subj_fpath)
            GZ_PEDS = " ".join([ACTOR.format(nm[-1], txt) for (nm, txt) in peds_fpath])

            RANDOM_HEIGHT = get_random_height_cmd(config["set_height"])

            # ==== TASK DISPATCH ====
            # =======================

            print("Dispatching background tasks.")

            sphinx = BackgroundTask(
                "sphinx {} {} {} {}".format(world_fpath, GZ_DRONE, GZ_SUBJECT, GZ_PEDS),
                log=False, wait=10
            )

            # RunTask("roslaunch bebop_driver bebop_node.launch", wait=5)
            # IlloKeWapo("roslaunch bebop_driver bebop_node.launch")

            bebop_driver = BackgroundTask(
                "roslaunch bebop_driver bebop_node.launch",
                log=False, shell=False, wait=12
            )
            
            print("Drone taking off!")
            RunTask("rostopic pub --once /bebop/takeoff std_msgs/Empty", wait=4)

            # if RANDOM_HEIGHT != "":
            #     print("Tweaking the drone altitude! hehe.")
            #     for i in range(MOVE_RNG):
            #         RunTask(MOVE_UP_DRONE.format(RANDOM_HEIGHT), wait=10)

            teleop = BackgroundTask(
                # "bash",
                "bash -e 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/bebop/cmd_vel'",
                shell=False, wait=0
            )

            # Run while teleop is active
            print("Starting to log data :D")

            data_logger = DataLogger(config, objects)
            data_logger.start()

            # ==== WAIT UNTIL EXPERT POLICY RECORDING IS FINISHED ====
            # ========================================================
            teleop.wait()

            data_logger.stop()  # saves-closes the .CSV when finished
            # bebop_driver.kill()
            sphinx.kill()
            os.system("pkill gzserver")


        # every once in a while delete ~/.parrot-sphinx logs
        # these files can get really big
        if random() > 0.9:
            _HOME = os.getenv("HOME")
            shutil.rmtree(_HOME + "/.parrot-sphinx")

        # roscore.kill()  # kills roscore after ALL runs

    except KeyboardInterrupt:
        print("\n ============")
        print("Sorry! We cannot exit this loop. Wait until it finishes :)")
        print("============\n")
        bebop_driver.kill()
        data_logger.stop()
        sphinx.kill()
        os.system("pkill ros")
        os.system("pkill gzserver")


# ==== MAIN FLOW ====
# ===================

if __name__ == "__main__":
    simulate(A_CONFIG)
