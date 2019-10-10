#!/usr/bin/python
# -*- coding: utf-8 -*-
# How the simulation generation should go

from __future__ import print_function
from utils.utils import RunTask, BackgroundTask, print_start, print_error, setupRun, get_random_height_cmd
from generators.world_builder import WorldBuilder
from datetime import datetime
from time import sleep

import sys
import signal

sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/aggregate")
sys.path.append("/home/pachacho/Documents/anafi_tools/envdata/telemetry")

# from master2 import MasterNode
from telemetry import TelemetryConsumer

# ====================

def stop_datalogger(sig, frame, logger):
    print("\nStopping the run\n")
    RunTask("rostopic pub --once /bebop/land std_msgs/Empty", wait=5)
    logger.stop()

def random_log_file(fdir, ext="csv"):
    rnd = datetime.now().strftime("%y%m%d_%H%M%S")
    return "{}/telemetry{}.{}".format(fdir, rnd, ext)


# ====================

RANDOM_HEIGHT = get_random_height_cmd()

TRAIN_NAME = "A_env"  # A = without peds

DATA_DIR = "/home/pachacho/Documents/anafi_tools/data/train"
DATA_RIGHT = "/1"
DATA_WRONG = "/0"

DRONE_FPATH = "/opt/parrot-sphinx/usr/share/sphinx/drones/local_bebop2.drone"
ACTOR = "/opt/parrot-sphinx/usr/share/sphinx/actors/pedestrian.actor::name={}::path={}"

CONFIG = {
    "runs": 1,
    "delay_start": 30,
    "object_probs": {"tree": 0.4, "door": 0.0, "wall": 0.2},
    "world_shape": (5,5),
    "number_peds": 1,
    "maximum_objects": 5,
    "subj_to_goal_dist": 4,
    "simulated": True,
    "model": None,
    "ip": None,
    "logfile": random_log_file(DATA_DIR, "csv")
}

# ====================

if __name__ == "__main__":
    
    for i in range(5):

        print("Run no {}".format(i))

        # telem = TelemetryConsumer(print, no_peds=1, objs=None)
        # telem.start()

        telem = BackgroundTask(
            "python home/pachacho/Documents/anafi_tools/envdata/telemetry/telemetryd.py",
            shell=False, daemon=True, stdout=False, wait=1
        )
        
        sleep(5)
        print("out")
        telem.kill()

    # for i in range(CONFIG["runs"]):
    #     print_start("Starting run no. {}".format(i))

    #     world = WorldBuilder(CONFIG)
    #     objects = world.get_object_models()

    #     world_fpath, subj_fpath, peds_fpath = world.get_paths()

    #     subject = ACTOR.format("subject", subj_fpath)
    #     # nm[-1] gives the pedestrian number; nm ~= P1, P0, ..
    #     pedestrians = " ".join([ACTOR.format(nm[-1], txt) for (nm, txt) in peds_fpath])

    #     # ====== START THE TASKS ======
    #     # ==== Total: 30s waiting =====
    #     # =============================

    #     BackgroundTask(
    #         "sphinx {} {} {} {}".format(world_fpath, DRONE_FPATH, subject, pedestrians),
    #         log=True, log_file="", wait=5
    #     )

    #     BackgroundTask("roscore", log=True, log_file="", wait=5)
        
    #     BackgroundTask(
    #         "roslaunch bebop_driver bebop_node.launch",
    #         log=True, log_file="", wait=10
    #     )
        
    #     RunTask("rostopic pub --once /bebop/takeoff std_msgs/Empty", wait=7)

    #     for i in range(5):
    #         RunTask(
    #             "rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist {}".format(RANDOM_HEIGHT),
    #             wait=3
    #         )

    #     teleop = BackgroundTask(
    #         "terminator -e 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/bebop/cmd_vel'",
    #         shell=True, wait=0.5
    #     )

    #     # Run while teleop is active
    #     data_logger = MasterNode(CONFIG, teleop)

    #     signal.signal(signal.SIGINT, lambda s,f: stop_datalogger(s,f,data_logger))
        
    #     data_logger.start()
