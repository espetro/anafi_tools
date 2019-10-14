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
import numpy as np
import pandas as pd


# =========================================================
# GLOBAL CONSTANTS AND SETUP
# =========================================================

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

DEFAULT_FORCE_CONFIG = {
    "Ar,o": 1.0,
    "Br,o": 1.0,
    "dR,o": 0.0,
    "Ar,h": 1.0,
    "Br,h": 1.0,
    "dR,h": 0.0,
}  # this way the constant multiplier is 1.0


# =========================================================
# FORCE CLASS DEFINITION
# =========================================================

class Forces:
    """A class to compute social attractive/repulsive forces"""
    def __init__(self, config=DEFAULT_FORCE_CONFIG, engine=None):
        """
        Setups the force computing environment.

        :param drone: A DroneModel object
        :param subject: A SubjectModel object
        :param peds: A dictionary of PedestrianModel objects
        :param objs: A list of object models, like WallBoxShape
        :param config: A dictionary of hyperparameters
        :param engine: The neural engine to be used. If None, no engine is used
        """
        self.config = config
        self.force_obj_cnt = config["Ar,o"] * np.exp(config["dR,o"] / config["Br,o"])
        self.force_ped_cnt = config["Ar,h"] * np.exp(config["dR,h"] / config["Br,h"])
        
        self.engine = engine
        self.drone, self.subject, self.pedestrians, self.objects = [None, None, None, None]
        self.forces = [0.0, 0.0, 0.0, 0.0]

    def predict_goal_force(self, drone_pos, subject, engine):
        """Predicts the goal force given the drone and subject poses"""
        force = 0.0
        if engine is None:
            pass
        else:
            nxt_pos = engine.fit(subject.get_pos())
            force = SubjectModel(*nxt_pos).distance_to(drone_pos)
        return force

    def get_peds_force(self, drone_pos, peds):
        """Computes the human repulsive force"""
        acc = 0
        norm_dr_pos = np.array(drone_pos) / np.linalg.norm(drone_pos)

        for (_, model) in peds.items():
            norm_obj_pose = np.array(model.pose) / np.linalg.norm(model.pose)
            acc += np.linalg.norm(norm_obj_pose - norm_dr_pos) * self.force_ped_cnt
        return acc

    def get_objs_force(self, dr_pos, objs):
        """Computes the object repulsive force"""
        acc = 0
        norm_dr_pos = np.array(dr_pos) / np.linalg.norm(dr_pos)

        for o in objs:
            norm_obj_pose = np.array(o.pose) / np.linalg.norm(o.pose)
            acc += np.linalg.norm(norm_obj_pose - norm_dr_pos) * self.force_obj_cnt
        return acc

    def update_forces(self, data):
        """
        Updates the forces given the current world state
        
        :param data: A dict with keys {drone, subject, peds, objs}
        :returns: A tuple with (companion, goal, peds, objs) forces
        """
        drone_pos = data["drone"].get_pos()

        comp_attr_force = data["subject"].distance_to(drone_pos)
        goal_attr_force = self.predict_goal_force(drone_pos, data["subject"], self.engine)

        ped_rep_force, obj_rep_force = [0.0, 0.0]
        if data["peds"] is not None:
            ped_rep_force = self.get_peds_force(drone_pos, data["peds"])
        if data["objs"] is not None:
            obj_rep_force = self.get_objs_force(drone_pos, data["objs"])
        
        self.forces = (comp_attr_force, goal_attr_force, ped_rep_force, obj_rep_force)

    def get_forces(self):
        """Returns all the computed forces"""
        return self.forces


# =========================================================
# DATABAG CLASS DEFINITION
# =========================================================

class DataBag:
    """Manages the data received by the sensors"""
    def __init__(self, no_peds=0, objs=None):
        """
        Setups empty model objects for the environment.
        Bear in mind some simulations cannot contain neither peds nor objs.

        :param no_peds:
        :param objs:
        """
        self.stop_flag = False
        self.drone = DroneModel()
        self.subject = SubjectModel()
        
        if no_peds > 0:
            self.peds = {str(i): PedestrianModel() for i in range(no_peds)}
        else:
            self.peds = None
        
        self.objs = objs

    def is_full(self):
        """Checks if the bag is full i.e. all objects are complete"""
        if not self.stop_flag:
            core_full = self.drone.complete() and self.subject.complete()
            if self.peds is None:
                return core_full
            else:
                return core_full and all([p.complete() for p in self.peds.values()])
        return False

    def reset(self):
        """Reset all the environment models"""
        self.drone.reset()
        self.subject.reset()
        if self.peds is not None:
            for _, model in self.peds.items():
                model.reset()

    def get_data(self):
        """Wraps the stored environment data into a dictionary and reset the data"""
        return {
            "ts": self.drone.ts,  # get the drone timestamp
            "drone": self.drone,
            "subject": self.subject,
            "peds": self.peds,
            "objs": self.objs
        }

        self.reset()

    def print_data(self):
        """Prints data in a human-readable way"""
        data = self.get_data()
        for (k,v) in data.items():
            print(k, v)


# =========================================================
# DATALOGGER CLASS DEFINITION
# =========================================================

class DataLogger:
    """Establishes the connection rules with the telemetry daemon"""
    def __init__(self, config, objs=None):
        """
        Setups a DataBag, TelemetryApp and Forces instances and creates an
        in-memory .csv file to store and later process the generated data.
        
        :param config: A dictionary following the project configuration guide.
        :param objs:
        """
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

        titles = [
            "ts", "dr_pos", "dr_vel", "dr_acc", "sub_pos",
            "goal_force", "subj_force", "ped_force", "obj_force"
        ] + peds_titles + objs_titles

        self.csv_writer.writerow(titles)

        # Configure data store and topics to watch
        self.PEDESTRIAN_TOPICS = DataLogger.get_ped_topics(config["final_peds"])
        self.bag = DataBag(config["final_peds"], objs)

        # Create a force computing node
        self.forces = Forces()

        # Setup the daemon
        self.daemon = App(config["run_name"], self.store_in_bag, config["sample_rate"])  
        
    def start(self):
        """Starts the telemetry daemon"""
        print("\nStarting daemon!")
        self.daemon.start()

    def stop(self):
        """Stops the whole application: TelemetryApp, DataBag and Forces"""
        self.bag.reset()
        self.bag.stop_flag = True

        self._preproc_file()
        self.file.close()
        self.daemon.stop()
        
    def store_in_bag(self, data):
        """
        Filter and group the telemetry data samples in different stores,
        and run a processing action if the databag is full for the given second.
        All the data objects stored should have the same timestamp and no None values.

        :param data: A dictionary with keys {ts, topic, namespace, coord, value, pid}
        """
        if random() > 0.99999:
            # check the incoming data once in a while
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

        if self.bag.is_full():
            data = self.bag.get_data()
            self.on_full(data)

    def on_full(self, bag_data):
        """
        Run an action when the databag is full for the given second i.e. no data
        value is None in any object and the timestamp is the same.
        Remember to check on pedestrian and object data, as they can be empty
        for a given simulation.

        :param bag_data: A dict with keys {ts, drone, subject, peds, objs}
        """
        dr_pos, dr_vel, dr_acc = bag_data["drone"].get_data()
        subj_pos = bag_data["subject"].get_pos()

        peds_poses, objs_poses = [], []
        if bag_data["peds"] is not None:
            peds_poses = [p.get_pos() for p in bag_data["peds"].values()]
        if bag_data["objs"] is not None:
            objs_poses = [m.pose for m in bag_data["objs"]]

        # self.forces.update_forces(bag_data)
        # goal_force, subj_force, ped_force, obj_force = self.forces.get_forces()

        goal_force, subj_force, ped_force, obj_force = [0.0, 0.0, 0.0, 0.0]

        if not self.file.closed:
            rowdata = [ 
                bag_data["ts"], dr_pos, dr_vel, dr_acc, subj_pos,+
                goal_force, subj_force, ped_force, obj_force
            ] + peds_poses + objs_poses

            self.csv_writer.writerow(rowdata)

    def _preproc_file(self):
        """Shrinks considerably the generated data to hold a unique row for each second"""
        self.file.seek(0)

        df = pd.read_csv(self.file, sep=",")
        df.drop_duplicates(subset=["ts"], inplace=True)
        df.reset_index(drop=True, inplace=True)
        df.to_csv(self.fname, sep=",", encoding="utf-8", index=False)

    @staticmethod
    def get_ped_topics(n_peds):
        """Get the list of pedestrian topics"""
        ls = []
        for n in range(n_peds):
            ls += [coord.format(n) for coord in PED_TEMPL_TOPICS]
        return ls