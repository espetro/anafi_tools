# -*- coding: utf-8 -*-
from __future__ import print_function, absolute_import
from subprocess import Popen, STDOUT, PIPE
from termcolor import colored
from random import randint
from time import sleep

import os
import shlex

# ======= CONFIGURATION =======
# =============================

TWIST_DN = "{linear: {x: 0.0, y: 0.0, z: -1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
TWIST_UP = "{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"

def setupRun():
    """Setups the options for the simulation"""
    return None

def print_start(_str):
    print(colored(_str, "green"))

def print_error(_str):
    print(colored(_str, "red"))

def get_random_height_cmd():
    x = randint(0,1)
    return [TWIST_DN, TWIST_UP][x]

# ======= SUBPROCESSES / THREADS =======
# ======================================

class BackgroundTask:
    """Launch a non-blocking task in the background"""
    def __init__(self, cmd, shell=False, log=False, stdout=False, daemon=False, log_file=None, wait=0):
        
        self.log = log

        if log is True:
            self.out = open(log_file, "w+")
        elif stdout is True:
            self.out = STDOUT
        else:
            self.out = PIPE

        sleep(wait)

        self.cmd = {
            True: cmd,
            False: shlex.split(cmd)
        }.get(shell)

        self.process = Popen(
            self.cmd,
            stdout=self.out,
            stderr=STDOUT,
            shell=shell,
            bufsize=1
        )

        self.process.setDaemon(daemon)

    def poll(self):
        """Returns None if the process is still running"""
        return self.process.poll()

    def kill(self):
        if self.log:
            self.out.close()

        self.process.kill()


class RunTask:
    """Launch a task that blocks the main app flow"""
    def __init__(self, cmd, wait=0):

        sleep(wait)
        os.system(cmd)

