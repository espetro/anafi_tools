# -*- coding: utf-8 -*-

from __future__ import print_function, absolute_import
from telemetry.pedestrian_model import PedestrianModel
from telemetry.subject_model import SubjectModel
from telemetry.drone_model import DroneModel
from telemetry.telemetryd import App
from random import randint, random
from datetime import datetime
from io import StringIO

import csv
import signal
import shutil
import threading
import pandas as pd


# ==== GLOBAL CONSTANTS AND SETUP ====
# ====================================

PED_TEMPL_TOPICS = [
    "omniscient_pedestrian{}.worldPosition.x", "omniscient_pedestrian{}.worldPosition.y",
    "omniscient_pedestrian{}.worldPosition.z",
]

DRONE_POS_TOPICS = [
    "omniscient_bebop2.worldPosition.x", "omniscient_bebop2.worldPosition.y",
    "omniscient_bebop2.worldPosition.z", "omniscient_anafi4k.worldPosition.x",
    "omniscient_anafi4k.worldPosition.y", "omniscient_anafi4k.worldPosition.z"
]

DRONE_VEL_TOPICS = [
    "omniscient_bebop2.relativeLinearVelocity.x", "omniscient_bebop2.relativeLinearVelocity.y",
    "omniscient_bebop2.relativeLinearVelocity.z", "omniscient_anafi4k.relativeLinearVelocity.x",
    "omniscient_anafi4k.relativeLinearVelocity.y", "omniscient_anafi4k.relativeLinearVelocity.z"
]

DRONE_ACC_TOPICS = [
    "omniscient_bebop2.relativeLinearAcceleration.x", "omniscient_bebop2.relativeLinearAcceleration.y",
    "omniscient_bebop2.relativeLinearAcceleration.z", "omniscient_anafi4k.relativeLinearAcceleration.x",
    "omniscient_anafi4k.relativeLinearAcceleration.y", "omniscient_anafi4k.relativeLinearAcceleration.z"
]

SUBJECT_TOPICS = [
    "omniscient_subject.worldPosition.x", "omniscient_subject.worldPosition.y",
    "omniscient_subject.worldPosition.z"
]

# ==== DATA LOGGER CLASS DEFINITIONS ====
# =======================================

class DataBag:
    """Manages the data received by the sensors"""
    def __init__(self, no_peds=0, objs=None):
        self.stop_flag = False
        self.drone = DroneModel()
        self.subject = SubjectModel()
        
        # Bear in mind some simulations cannot contain neither peds nor objs
        if no_peds > 0:
            self.peds = {str(i): PedestrianModel() for i in range(no_peds)}
        else:
            self.peds = None
        
        self.objs = objs

    def is_full(self):
        if not self.stop_flag:
            # Bear in mind some simulations cannot contain neither peds nor objs
            core_full = self.drone.complete() and self.subject.complete()
            if self.peds is None:
                return core_full
            else:
                return core_full and all([p.complete() for p in self.peds.values()])
        return False

    def reset(self):
        self.drone.reset()
        self.subject.reset()
        if self.peds is not None:
            for _, model in self.peds.items():
                model.reset()

    def get_data(self):
        return {
            "ts": self.drone.pos[0][0],  # get the drone timestamp
            "drone": self.drone,
            "subject": self.subject,
            "peds": self.peds,
            "objs": self.objs
        }

        self.reset()

    def print_data(self):
        data = self.get_data()
        for (k,v) in data.items():
            print(k, v)


class DataLogger:
    """Establishes the connection rules with the telemetry daemon"""
    def __init__(self, config, objs=None):
        # Configure data logging in .csv
        self.fname = "{}/{}{}.csv".format(
            config["datapath"],
            config["name"],
            datetime.now().strftime("%y%m%d_%H%M%S")
        )
        # Opens an in-memory file to do later processing
        self.file = StringIO()
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

        # Configure data store and topics to watch
        self.PEDESTRIAN_TOPICS = DataLogger.get_ped_topics(config["final_peds"])
        self.bag = DataBag(config["final_peds"], objs)

        # Setup the daemon
        self.daemon = App(config["run_name"], self.store_in_bag, config["sample_rate"])  
        
    def start(self):
        print("\nStarting daemon!")
        self.daemon.start()

    def stop(self):
        self.bag.reset()
        self.bag.stop_flag = True

        self._preproc_file()
        self.file.close()
        self.daemon.stop()
        
    def store_in_bag(self, data):
        """Store data samples sent in multiple batches"""
        # print("Bag logging")

        if random() > 0.9999:
            print("Telemetry data: ", data["topic"])
            print("Bag data: ", self.bag.print_data())
            
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

        if self.bag.is_full():  # dude this is wrong, it even writes None data if no telem proc is killed
            # Ensure that all data have the same timestamp and are not None
            # print("BAG DATA FULL")

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

            self.csv_writer.writerow(rowdata)

    def _preproc_file(self):
        """Shrinks considerably the generated data to hold a unique row for each second"""
        self.file.seek(0)

        df = pd.read_csv(self.file, sep=",")
        df.drop_duplicates(subset=["ts"], inplace=True)
        df.reset_index(drop=True, inplace=True)
        df.to_csv(self.fname, sep=",", encoding="utf-8", index=False)

        # don't need it but dude it's good sauce
        # with open(self.fname, "w") as f:
        #     self.file.seek(0)
        #     shutil.copyfileobj(self.file, f, -1)

    @staticmethod
    def get_ped_topics(n_peds):
        """Get the list of pedestrian topics"""
        ls = []
        for n in range(n_peds):
            ls += [coord.format(n) for coord in PED_TEMPL_TOPICS]
        return ls