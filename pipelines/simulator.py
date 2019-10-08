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

    DRN = "/opt/parrot-sphinx/usr/share/sphinx/drones/local_bebop2.drone"
    ACTOR = "/opt/parrot-sphinx/usr/share/sphinx/actors/pedestrian.actor::name={}::path={}"
    SUBJ = ACTOR.format("subject", SUBJ_PATH)
    PED1 = ACTOR.format("pedestrian0", PED0_PATH)

    sphinx = Popen(
        shlex.split("sphinx {} {} {} {}".format(WORLD_FILE, DRN, SUBJ, PED1)),
        stdout=None,
        stderr=None,
        bufsize=1
    )


    false_world_objects = [FalseModel() for j in range(10)]

    telem = Telemetry(CSV_PATH, false_world_objects, 1, 1000)

    # telem = threading.Thread(
    #     name="telemetry_consumer",
    #     target=Telemetry,
    #     args=(CSV_PATH, false_world_objects, 1, 1000)
    # )

    try:
        while sphinx.poll() is None:
            print("============ Go on ============")
            telem.looper.stepLoop()
            # print("Press CTRL+C if you fancy! Everything is running in the bg.")
            sleep(0.5)
    except KeyboardInterrupt as e:
        print(e)
        telem.looper.exitLoop()
        telem.stop()
        os.system("pkill python")
        sphinx.kill()

    # Popen(roscore)
    # Popen(ros bebop_autonomy)
    # Popen(takeoff)
    # os.system("ros teleop") # and wait for it to be killed, CTRL+Ced