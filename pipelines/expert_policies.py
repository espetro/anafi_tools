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
import signal
import shutil

# Not needed if control and envdata packages are pip-installed
# sys.path.append("/home/pachacho/Documents/anafi_tools/control")
# sys.path.append("/home/pachacho/Documents/anafi_tools/envdata")

try:
    import olympe
    from telemetry.data_logger import DataLogger
    from teleop.joystick import JoystickTeleop
except ImportError:
    print("Olympe has not been loaded yet! Cannot run the app", file=sys.__stderr__)
    sys.exit(0)



# ==== FUNCTIONS =======
# ====================

class Simulator:

    def __init__(self, config):
        """Runs a simulation given the configuration dictionary"""
        signal.signal(signal.SIGTERM, self._killall)
        self.MOVE_RNG = 5

        self.SPHINXRT = os.getenv("SPHINX_ROOT")
        self.ACTOR = self.SPHINXRT + "/actors/pedestrian.actor::name={}::path={}"
        self.GZ_DRONE = self.SPHINXRT + "/drones/local_anafi4k.drone"

        self.teleop, self.sphinx, self.data_logger = [None, None, None]
        self.num_early_stops = 0
        self.num_iters = 0
        
        self.config = config

    def start(self):
        self._mainloop(self.config)

    def vary_anafi_height(self, u_sure):
        if u_sure:
            val = [0.5, -0.5][randint(0,1)]
            for _ in range(self.MOVE_RNG):
                self.teleop.move(0, 0, 0, val)

    def _killall(self, sig, frame):
        print("Stopping the whole simulation")
        self._kill_if_open()
        sys.exit(0)

    def _kill_if_open(self):
        if self.teleop is not None:
            self.teleop.stop()
        if self.data_logger is not None:
            self.data_logger.stop()
        if self.sphinx is not None:
            self.sphinx.kill(); os.system("pkill gzserver")

    @staticmethod
    def del_sphinx_logs():
        """Periodically delete ~/.parrot-sphinx logs"""
        if random() > 0.95:
            _HOME = os.getenv("HOME")
            shutil.rmtree(_HOME + "/.parrot-sphinx")
            
    @staticmethod
    def kill_telemetry_orphans():
        """Remove telemetry daemons left as orphans in the exec. queue"""
        os.system("pkill telem")

    def _mainloop(self, config):
        """"""
        while self.num_iters < config["reps"]:
            Simulator.kill_telemetry_orphans()
            Simulator.del_sphinx_logs()
            
            try:
                self.teleop, self.sphinx, self.data_logger = [None, None, None]

                config["run_name"] = "run{}".format(self.num_iters)
                print("\nRun {}. Initial wait: {}s. Early stop at {}s".format(
                    self.num_iters, config["delay_start"], config["early_stop"]
                ))


                print("Generating a random world and setting a path")
                world = WorldBuilder(config)
                objects = world.get_object_models()
                (world_fpath, subj_fpath, peds_fpath) = world.get_paths()
                config["final_peds"] = world.get_num_peds()


                if world.path2list() != []:
                    GZ_SUBJECT = self.ACTOR.format("subject", subj_fpath)
                    GZ_PEDS = " ".join([
                        self.ACTOR.format(nm[-1], txt) for (nm, txt) in peds_fpath
                    ])


                    print("Launching Parrot Sphinx.")
                    sphinx_fname = config["datapath"] + "/sphinx{}.log".format(
                        self.num_iters
                    )
                    sph_str = "sphinx --datalog {} {} {} {}".format(
                        world_fpath, self.GZ_DRONE, GZ_SUBJECT, GZ_PEDS
                    )
                    self.sphinx = BackgroundTask(
                        sph_str,
                        log=True, log_file=sphinx_fname, wait=8
                    )


                    print("Seting up the teleop")
                    self.teleop = JoystickTeleop(
                        olympe.Drone(JoystickTeleop.SIMULATED_IP, loglevel=0),
                        config["speed"],
                        config["refresh"]
                    )
                    self.vary_anafi_height(config["set_height"])
                    self.teleop._takeoff()


                    print("Running the data logger store")
                    self.data_logger = DataLogger(config, objects)
                    self.data_logger.start()


                    print("Starting the teleop")
                    self.teleop.start()

                    print("Sleepin'")
                    sleep(3)

                    print("\nNICE RUN!\n")
                    self._kill_if_open()
                    self.num_iters += 1             
                else:
                    print("This run was early-stopped")
                    self.num_early_stops += 1
                    print("Total early stops: {}".format(self.num_early_stops))
                    print("Running an extra iteration")


            except KeyboardInterrupt:
                print("\nKeyboardInterruption! Running an extra iteration\n")
                self._kill_if_open()
                continue

        print("Opening an explorer window")        
        RunTask("nautilus {} &".format(config["datapath"]))



# ==== MAIN FLOW ====
# ===================

if __name__ == "__main__":
    sim = Simulator(A_CONFIG)
    sim.start()