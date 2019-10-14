# -*- coding: utf-8 -*-
from __future__ import print_function, absolute_import
from telemetry.pedestrian_model import PedestrianModel
from telemetry.subject_model import SubjectModel
from telemetry.drone_model import DroneModel
from telemetry.telemetryd import App
from random import randint, random
from datetime import datetime

import csv
import signal

# ==== GLOBAL CONSTANTS AND SETUP ====
# ====================================

PED_TEMPL_TOPICS = [
    "omniscient_pedestrian{}.worldPosition.x",
    "omniscient_pedestrian{}.worldPosition.y",
    "omniscient_pedestrian{}.worldPosition.z",
]

DRONE_POS_TOPICS = [
    "omniscient_bebop2.worldPosition.x",
    "omniscient_bebop2.worldPosition.y",
    "omniscient_bebop2.worldPosition.z",
    "omniscient_anafi4k.worldPosition.x",
    "omniscient_anafi4k.worldPosition.y",
    "omniscient_anafi4k.worldPosition.z"
]

DRONE_VEL_TOPICS = [
    "omniscient_bebop2.relativeLinearVelocity.x",
    "omniscient_bebop2.relativeLinearVelocity.y",
    "omniscient_bebop2.relativeLinearVelocity.z",
    "omniscient_anafi4k.relativeLinearVelocity.x",
    "omniscient_anafi4k.relativeLinearVelocity.y",
    "omniscient_anafi4k.relativeLinearVelocity.z"
]

DRONE_ACC_TOPICS = [
    "omniscient_bebop2.relativeLinearAcceleration.x",
    "omniscient_bebop2.relativeLinearAcceleration.y",
    "omniscient_bebop2.relativeLinearAcceleration.z",
    "omniscient_anafi4k.relativeLinearAcceleration.x",
    "omniscient_anafi4k.relativeLinearAcceleration.y",
    "omniscient_anafi4k.relativeLinearAcceleration.z"
]

SUBJECT_TOPICS = [
    "omniscient_subject.worldPosition.x",
    "omniscient_subject.worldPosition.y",
    "omniscient_subject.worldPosition.z"
]

# ==== DATA LOGGER CLASS DEFINITIONS ====
# =======================================

class DataBag:
    """Manages the data received by the sensors"""
    def __init__(self, no_peds=0, peds_topics=[], num_s_samples=1, objs=None):
        """Bear in mind some simulations cannot contain neither peds nor objs"""
        self.global_ts = -1
        self.PEDESTRIAN_TOPICS = peds_topics

        self.drone = DroneModel(num_s_samples)
        self.subject = SubjectModel(num_s_samples)
        
        if no_peds > 0:
            self.peds = {
                str(i): PedestrianModel(num_s_samples) for i in range(no_peds)
            }
        else:
            self.peds = None
        
        self.objs = objs

    def is_coord_empty(self, data):
        """Check if the given data is not filled already in the bag"""
        check = False
        if data["topic"] in DRONE_POS_TOPICS:
            check = self.drone.check_if_pos(data["coord"])
        elif data["topic"] in DRONE_VEL_TOPICS:
            check = self.drone.check_if_vel(data["coord"])
        elif data["topic"] in DRONE_ACC_TOPICS:
            check = self.drone.check_if_acc(data["coord"])
        elif data["topic"] in SUBJECT_TOPICS:
            check = self.subject.check_if_pos(data["coord"])
        elif data["topic"] in self.PEDESTRIAN_TOPICS:
            check = self.peds[data["pid"]].check_if_pos(data["coord"])
        return check

    def add(self, data):
        """Stores the given data"""
        if data["topic"] in DRONE_POS_TOPICS:
            self.drone.set_pos_val(data["ts"], data["coord"], data["value"])
        elif data["topic"] in DRONE_VEL_TOPICS:
            self.drone.set_vel_val(data["ts"], data["coord"], data["value"])
        elif data["topic"] in DRONE_ACC_TOPICS:
            self.drone.set_acc_val(data["ts"], data["coord"], data["value"])
        elif data["topic"] in SUBJECT_TOPICS:
            self.subject.set_val(data["ts"], data["coord"], data["value"])
        elif data["topic"] in self.PEDESTRIAN_TOPICS:
            self.peds[data["pid"]].set_val(data["ts"], data["coord"], data["value"])

    def is_full(self):
        """Check if all models stored are complete for the given timestamp."""
        core_full = self.drone.complete() and self.subject.complete()
        if self.peds is None:
            return core_full
        else:
            return core_full and all([p.complete() for p in self.peds.values()])

    def empty_bag(self):
        """Empties the models within the bag"""
        if self.peds is not None:
            for _, model in self.peds.items():
                model.reset()
        self.drone.reset()
        self.subject.reset()

    def get_data(self):
        """Flushes the stored data and empties the bag. Use the drone ts"""
        data = {
            "ts": self.drone.pos[0][0],
            "drone": self.drone,
            "subject": self.subject,
            "peds": self.peds,  # can be None
            "objs": self.objs   # can be None
        }
        self.empty_bag()
        return data

    def print_data(self):
        data = self.get_data()
        for (k,v) in data.items():
            print(k, v)


# ======================================================
# ======================================================

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

        peds_titles = []
        if config["final_peds"] > 0:
            peds_titles = ["pedestrian{}_pos".format(n) for n in range(config["final_peds"])]

        objs_titles = []
        if objs is not None:
            objs_titles = ["{}_pos".format(model.oid) for model in objs]

        print("\nPed titles: ", peds_titles)
        print("\nObj titles: ", objs_titles)

        titles = ["ts", "dr_pos", "dr_vel", "dr_acc", "sub_pos", "force_goal", "force_ped", "force_obj"] + peds_titles + objs_titles
        self.csv_writer.writerow(titles)

        # Configure pedestrian topics to watch
        ped_topics = DataLogger.get_ped_topics(config["final_peds"])

        # Configure data (temporal) bag
        self.bag = DataBag(
            config["final_peds"],
            ped_topics,
            config["samples_per_s"],
            objs
        )

        # Setup the daemon
        self.daemon = App(config["run_name"], self.store_in_bag, config["sample_rate"])  
        
    def start(self):
        print("\nStarting daemon!")
        self.daemon.start()

    def stop(self):
        self.file.close()
        self.daemon.stop()
        
    def store_in_bag(self, data):
        """Store data samples sent in multiple batches"""
        # timestamp is (s, nanos): data["ts"], data["tnanos"]

        self.bag.add(data)

        # Ensure that all data have the same timestamp and are not None
        # Also there can't be more than a sample per second.
        if self.bag.is_full():
            if random() > 0.99999:
                print("Telemetry data: ", data["topic"])
                print("Bag data: ", self.bag.print_data())

            # Then flush the data to process it and empty the bag
            data = self.bag.get_data()
            self.on_full(data)

    def on_full(self, bag_data):
        """Do computations when multiple samples with equal timestamp are received"""
        # bag_data is a dict {ts, drone, subject, peds, objs}
        # Bear in mind some simulations cannot contain neither peds nor objs
        dr_pos, dr_vel, dr_acc = bag_data["drone"].get_data()
        subj_pos = bag_data["subject"].get_pos()

        peds_poses = []
        if bag_data["peds"] is not None:
            peds_poses = [p.get_pos() for p in bag_data["peds"].values()]
        objs_poses = []
        if bag_data["objs"] is not None:
            objs_poses = [m.pose for m in bag_data["objs"]]

        # Bear in mind some simulations cannot contain neither peds nor objs
        # (ped and obj force would be 0.0)
        forces = [0,0,0]

        if not self.file.closed:
            rowdata = [ 
                bag_data["ts"], dr_pos, dr_vel, dr_acc, subj_pos, forces[0], forces[1], forces[2]
            ] + peds_poses + objs_poses

            # print("\n\nROWDATA: ", rowdata, "\n\n")
            self.csv_writer.writerow(rowdata)

    @staticmethod
    def get_ped_topics(n_peds):
        """Get the list of pedestrian topics"""
        ls = []
        for n in range(n_peds):
            ls += [coord.format(n) for coord in PED_TEMPL_TOPICS]
        return ls