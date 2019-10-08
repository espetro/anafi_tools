#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function, absolute_import
from streaming.simulation.world_parser import WorldParser
from streaming.simulation.telemetry import Telemetry
from subprocess import Popen, STDOUT, PIPE
from random import randint
from time import sleep

import os
import sys
import shlex
import signal
import datetime
import threading


class FalseModel:
    def __init__(self):
        self.pos = FalseModel.random_pos()
        self.type = "ttype"

    @staticmethod
    def random_pos():
        return [randint(0,10) for i in range(3)]

# importing from non-package submodules doesnt work for "python folder/sim.py"
# run from anafi_tools/ as "python -m pipelines.simulator"

if __name__ == "__main__":
    """Testing the to-be simulation environment"""
    sys.path.append("..")

    DIR = os.path.expanduser("~") + "/Documents/anafi_tools/generators/example"
    CSV_DIR = os.path.expanduser("~") + "/Documents/anafi_tools/data/example"
    CSV_PATH = "{}/example{}.csv".format(CSV_DIR, datetime.datetime.now().strftime("%y%m%d_%H%M%S"))

    WORLD_FILE = DIR + "/tmpBXsMv8.world"
    PED0_PATH = DIR + "/tmp12QhIR.path"
    SUBJ_PATH = DIR + "/tmpvGsKxl.path"

    world_objects = WorldParser(WORLD_FILE)
    [print(x) for x in world_objects.objects]
    print()
    [print(x) for x in world_objects.models]

    # DRN = "/opt/parrot-sphinx/usr/share/sphinx/drones/local_bebop2.drone"
    # ACTOR = "/opt/parrot-sphinx/usr/share/sphinx/actors/pedestrian.actor::name={}::path={}"
    # SUBJ = ACTOR.format("subject", SUBJ_PATH)
    # PED1 = ACTOR.format("pedestrian0", PED0_PATH)

    # sphinx_file = open(CSV_DIR + "/sphinx.log", "w+")
    # roscore_file = open(CSV_DIR + "/roscore.log", "w+")
    # bebop_file = open(CSV_DIR + "/bebop.log", "w+")

    # sphinx = Popen(
    #     shlex.split("sphinx {} {} {} {}".format(WORLD_FILE, DRN, SUBJ, PED1)),
    #     stdout=sphinx_file,
    #     stderr=None,
    #     bufsize=1
    # )


    # false_world_objects = [FalseModel() for j in range(10)]


    # print("Starting ROScore...", file=sys.stderr)
    # roscore = Popen(
    #     shlex.split("roscore"),
    #     stdout=roscore_file,
    #     stderr=STDOUT,
    #     bufsize=1
    # )

    # sleep(10)
    # print("Setting up drone ROS driver...", file=sys.stderr)
    # bebop_node = Popen(
    #     shlex.split("roslaunch bebop_driver bebop_node.launch"),
    #     stdout=bebop_file,
    #     stderr=STDOUT,
    #     bufsize=1
    # )

    # sleep(7)
    # print("Taking off...", file=sys.stderr)
    # take_off = os.system("rostopic pub --once /bebop/takeoff std_msgs/Empty")

    # # # go up or down 0.5m, or stays
    # print("Setting up drone height...", file=sys.stderr)
    # topic_pub = "rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist"
    # twist_dn = "{linear: {x: 0.0, y: 0.0, z: -1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
    # twist_up = "{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
    # height_drone = randint(-1,1)

    # if height_drone != 0:
    #     if height_drone == -1:
    #         my_twist = twist_dn
    #     elif height_drone == 1:
    #         my_twist = twist_up
        
    #     for i in range(5):    
    #         os.system("{} {}".format(topic_pub, my_twist))
    #         sleep(4)

    # print("Setting up telemetry consumer...", file=sys.stderr)
    # telem = Telemetry(CSV_PATH, false_world_objects, 1, 1000)

    # print("\n\n\n\n=========================")
    # print("There is life after the TELEMETRY")
    # print("\n\n\n\n=========================")

    # sleep(5)
    # try:
    #     os.system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/bebop/cmd_vel")
    # except KeyboardInterrupt as e:
    #     print("Teleop finished...", file=sys.stderr)

    # bebop_file.close()
    # roscore_file.close()
    # sphinx_file.close()

    # print("Landing the drone...", file=sys.stderr)
    # landing = os.system("rostopic pub --once /bebop/land std_msgs/Empty")
    # sleep(4)

    # bebop_node.kill()
    # roscore.kill()
    # sphinx.kill()
    # print("All subprocesses killed")
    # telem.stop()
    # print("Run finished")


    
    # try:
    #     os.system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/bebop/cmd_vel")
    #     # while sphinx.poll() is None:
    #         # telem.looper.stepLoop()
    #         # print("Press CTRL+C if you fancy! Everything is running in the bg.")
    #         # sleep(0.5)
    #         # print("hello")
    # except KeyboardInterrupt as e:
    #     print(e)

    #     print("Landing the drone...", file=sys.stderr)
    #     landing = os.system("rostopic pub --once /bebop/land std_msgs/Empty")
    #     bebop_node.kill()
    #     roscore.kill()
    #     sphinx.kill()
    #     telem.stop()

    #     # os.system("pkill python")

    # # Popen(roscore)
    # # Popen(ros bebop_autonomy)
    # # Popen(takeoff)
    # # os.system("ros teleop") # and wait for it to be killed, CTRL+Ced