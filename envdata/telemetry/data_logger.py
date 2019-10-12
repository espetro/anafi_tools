# -*- coding: utf-8 -*-

from __future__ import print_function
from drone_model import DroneModel
from subject_model import SubjectModel
from pedestrian_model import PedestrianModel
from random import randint, random
from datetime import datetime
from telemetryd import App

import csv
import signal
import threading


# ==== GLOBAL CONSTANTS AND SETUP ====
# ====================================

PED_TEMPL_TOPICS = [
    "omniscient_pedestrian{}.worldPosition.x", "omniscient_pedestrian{}.worldPosition.y",
    "omniscient_pedestrian{}.worldPosition.z",
]

DRONE_POS_TOPICS = [
    "omniscient_bebop2.worldPosition.x", "omniscient_bebop2.worldPosition.y",
    "omniscient_bebop2.worldPosition.z", "omniscient_anafi.worldPosition.x",
    "omniscient_anafi.worldPosition.y", "omniscient_anafi.worldPosition.z"
]

DRONE_VEL_TOPICS = [
    "omniscient_bebop2.relativeLinearVelocity.x", "omniscient_bebop2.relativeLinearVelocity.y",
    "omniscient_bebop2.relativeLinearVelocity.z", "omniscient_anafi.relativeLinearVelocity.x",
    "omniscient_anafi.relativeLinearVelocity.y", "omniscient_anafi.relativeLinearVelocity.z"
]

DRONE_ACC_TOPICS = [
    "omniscient_bebop2.relativeLinearAcceleration.x", "omniscient_bebop2.relativeLinearAcceleration.y",
    "omniscient_bebop2.relativeLinearAcceleration.z", "omniscient_anafi.relativeLinearAcceleration.x",
    "omniscient_anafi.relativeLinearAcceleration.y", "omniscient_anafi.relativeLinearAcceleration.z"
]

SUBJECT_TOPICS = [
    "omniscient_subject.worldPosition.x", "omniscient_subject.worldPosition.y",
    "omniscient_subject.worldPosition.z"
]

# ==== DATA LOGGER CLASS DEFINITIONS ====
# =======================================

class DataBag:
    """Manages the data received by the sensors"""
    def __init__(self, no_peds, objs):
        self.drone = DroneModel()
        self.subject = SubjectModel()
        self.peds = {str(i): PedestrianModel() for i in range(no_peds)}
        self.objs = objs

    def is_full(self):
        return self.drone.complete() and \
            self.subject.complete() and \
            all([p.complete() for p in self.peds.values()])

    def get_data(self):
        return {
            "ts": self.drone.pos[0][0],  # get the drone timestamp
            "drone": self.drone,
            "subject": self.subject,
            "peds": self.peds,
            "objs": self.objs
        }

    def print_data(self):
        data = self.get_data()
        for (k,v) in data.items():
            print(k, v)


class DataLogger:
    """Establishes the connection rules with the telemetry daemon"""
    def __init__(self, config, objs=None):
        # Configure data logging in .csv
        fname = "{}/{}{}.csv".format(
            config["datapath"],
            config["name"],
            datetime.now().strftime("%y%m%d_%H%M%S")
        )
        self.file = open(fname, "w+")
        self.csv_writer = csv.writer(self.file, delimiter=",")

        peds_titles = ["pedestrian{}_pos".format(n) for n in range(config["final_peds"])]
        objs_titles = ["{}_pos".format(model.oid) for model in objs]
        # for m in objs:
        #     objs_titles += [m.oid, ]

        self.csv_writer.writerow([
            "ts", "dr_pos", "dr_vel", "dr_acc", "sub_pos",
            "force_goal", "force_ped", "force_obj"
        ] + peds_titles + objs_titles)

        # Configure data store and topics to watch
        self.PEDESTRIAN_TOPICS = DataLogger.get_ped_topics(config["final_peds"])
        self.bag = DataBag(config["final_peds"], objs)

        # Setup the daemon
        self.daemon = App(config["run_name"], self.store_in_bag, config["sample_rate"])  
        
    def start(self):
        self.daemon.start()

    def stop(self):
        self.file.close()
        self.daemon.stop()
        
    def store_in_bag(self, data):
        """Store data samples sent in multiple batches"""
        if random > 0.9999:
            print(data["topic"])
            
        if data["topic"] in DRONE_POS_TOPICS:
            self.bag.drone.set_pos_val(data["ts"], data["coord"], data["value"])

        elif data["topic"] in DRONE_VEL_TOPICS:
            self.bag.drone.set_vel_val(data["ts"], data["coord"], data["value"])

        elif data["topic"] in DRONE_ACC_TOPICS:
            self.bag.drone.set_acc_val(data["ts"], data["coord"], data["value"])

        elif data["topic"] in SUBJECT_TOPICS:
            self.bag.subject.set_val(data["ts"], data["coord"], data["value"])

        elif data["topic"] in self.PEDESTRIAN_TOPICS:
            self.bag.peds[data["pid"]].set_val(data["ts"], data["coord"], data["value"])

        if self.bag.is_full():
            # Ensure that all data have the same timestamp and are not None
            data = self.bag.get_data()
            self.on_full(data)

    def on_full(self, bag_data):
        """Do computations when multiple samples with equal timestamp are received"""
        # bag_data is a dict {ts, drone, subject, peds, objs}
        dr_pos, dr_vel, dr_acc = bag_data["drone"].get_data()
        subj_pos = bag_data["subject"].get_pos()
        peds_poses = [p.get_pos() for p in bag_data["peds"].values()]
        objs_poses = [m.pose for m in bag_data["objs"]]

        forces = [0,0,0]

        if not self.file.closed:
            self.csv_writer.writerow([
                bag_data["ts"], dr_pos, dr_vel, dr_acc, subj_pos,
                forces[0], forces[1], forces[2],
            ] + peds_poses + objs_poses)

    @staticmethod
    def get_ped_topics(n_peds):
        """Get the list of pedestrian topics"""
        ls = []
        for n in range(n_peds):
            ls += [coord.format(n) for coord in PED_TEMPL_TOPICS]
        return ls