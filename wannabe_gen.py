#!/usr/bin/python
# -*- coding: utf-8 -*-
# How the simulation generation should go

from __future__ import print_function, absolute_import
from utils.utils import RunTask, BackgroundTask, print_start, print_error, setupRun
from generators.world_builder import WorldBuilder
from utils.configs import A_CONFIG
from random import random, randint
from time import time, sleep

import os
import sys
import shutil

try:
    import olympe
except ImportError:
    print("Olympe has not been loaded yet! Cannot run the app", file=sys.__stderr__)
    sys.exit(0)

sys.path.append("/home/pachacho/Documents/anafi_tools/control")
sys.path.append("/home/pachacho/Documents/anafi_tools/envdata")
from telemetry.data_logger import DataLogger
from teleop.joystick import JoystickTeleop


# ==== FUNCTIONS =======
# ====================

def vary_anafi_height(ctrl, u_sure, times):
    if u_sure:
        val = [0.5, -0.5][randint(0,1)]
        for _ in range(times):
            ctrl.move(0, 0, 0, val)


def simulate(config):
    """Runs a simulation given the configuration dictionary"""    

    SPHINX_ROOT = os.getenv("SPHINX_ROOT")
    ACTOR = SPHINX_ROOT + "/actors/pedestrian.actor::name={}::path={}"
    GZ_DRONE = SPHINX_ROOT + "/drones/local_anafi4k.drone"

    MOVE_RNG = 5  # moves the drone in the Z axis by 0.5 or -0.5

    try:
        teleop, sphinx, data_logger = None, None, None

        for i in range(config["reps"]):

            os.system("pkill telem")  # HIGHLY DISCOURAGED

            print("\nRun no {}. Initial wait: {}".format(i, config["delay_start"]))
            config["run_name"] = "run{}".format(i)

            # ==== PER-RUN SETUP ====        
            # =======================

            # Generate world files (depends on the grid size)
            print("Generating new world.")
            tstart = time()

            world = WorldBuilder(config)
            objects = world.get_object_models()
            (world_fpath, subj_fpath, peds_fpath) = world.get_paths()
            config["final_peds"] = world.get_num_peds()

            print("{} seconds to generate a world".format(int(time() - tstart)))

            # nm[-1] gives the pedestrian number; nm ~= P1, P0, ..
            GZ_SUBJECT = ACTOR.format("subject", subj_fpath)
            GZ_PEDS = " ".join([ACTOR.format(nm[-1], txt) for (nm, txt) in peds_fpath])


            # ==== TASK DISPATCH ====
            # =======================

            print("Dispatching background tasks.")

            sphinx_fname = config["datapath"] + "/sphinx{}.log".format(i)
            sphinx = BackgroundTask(
                "sphinx --datalog {} {} {} {}".format(world_fpath, GZ_DRONE, GZ_SUBJECT, GZ_PEDS),
                log=True, log_file=sphinx_fname, wait=8
            )

            # Create drone and joystick and randomly vary drone height
            # around 5s creating both
            drone = olympe.Drone(JoystickTeleop.SIMULATED_IP, loglevel=0)
            teleop = JoystickTeleop(drone, config["speed"], config["refresh"])
            
            vary_anafi_height(teleop, config["set_height"], MOVE_RNG)

            print("Drone taking off!")
            # about 4s
            teleop._takeoff()

            print("Starting to log data :D")
            # about 4s
            data_logger = DataLogger(config, objects)
            data_logger.start()

            # ==== WAIT UNTIL EXPERT POLICY RECORDING IS FINISHED ====
            # ========================================================
            # Control the drone :D
            teleop.start()

            print("\n\nKilling processes!")
            sphinx.kill(); os.system("pkill gzserver")
            # print("Halo")
            data_logger.stop()  # saves-closes the .CSV when finished


        # every once in a while delete ~/.parrot-sphinx logs
        # these files can get really big
        if random() > 0.9:
            _HOME = os.getenv("HOME")
            shutil.rmtree(_HOME + "/.parrot-sphinx")

        # roscore.kill()  # kills roscore after ALL runs

    except KeyboardInterrupt:
        print("\nStopping the whole simulation :(")
        if teleop is not None:
            teleop.stop()
        if data_logger is not None:
            data_logger.stop()
        if sphinx is not None:
            sphinx.kill(); os.system("pkill gzserver")


# ==== MAIN FLOW ====
# ===================

if __name__ == "__main__":
    cnf = A_CONFIG

    simulate(cnf)
    RunTask("nautilus {} &".format(cnf["datapath"]))