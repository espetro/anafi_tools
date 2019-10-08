# -*- coding: utf-8 -*-

from __future__ import print_function, absolute_import
from streaming.simulation.telemetryd import TelemetryConsumer
from streaming.simulation.drone import DroneModel
from streaming.simulation.subject import SubjectModel
from streaming.simulation.pedestrian import PedestrianModel

import csv
import signal
import threading

class Telemetry:
    DRONE_TOPICS = [
        "omniscient_bebop2.worldPosition.x",
        "omniscient_bebop2.worldPosition.y",
        "omniscient_bebop2.worldPosition.z",
        "omniscient_anafi.worldPosition.x",
        "omniscient_anafi.worldPosition.y",
        "omniscient_anafi.worldPosition.z"
    ]

    SUBJECT_TOPICS = [
        "omniscient_subject.worldPosition.x",
        "omniscient_subject.worldPosition.y",
        "omniscient_subject.worldPosition.z"
    ]

    PED_TEMPL_TOPICS = [
        "omniscient_pedestrian{}.worldPosition.x",
        "omniscient_pedestrian{}.worldPosition.y",
        "omniscient_pedestrian{}.worldPosition.z",
    ]

    def __init__(self, csv_path, world_objects, no_peds=0, rate=1000):
        """
        Simple interface to telemetryd. Setups a log in a .csv file with headers,
        extracts the parsed world object models, assigns the telemetry topics to
        each dynamic model (drone, subject and pedestrians) and runs the 
        telemetry consumer in foreground.

        :param csv_path:
        :param world_objects:
        :param no_peds:
        :param rate:
        :param ctrl_addr: Deprecated.
        :param dataport: Deprecated.
        """
        self.flags = {
            "drone": True, "peds": True, "subject": True
        }

        open(csv_path, "w").close()  # Cleans the previous log if any
        self.fpath = csv_path
        self.file = open(csv_path, "w+")
        self.csv_writer = csv.writer(self.file)
        
        signal.signal(signal.SIGINT, self.close_csv)

        csv_header = ["timestamp", "drone_pos", "subject_pos"]
        csv_header += ["pedestrian{}_pos".format(i) for i in range(no_peds)]
        for j in range(len(world_objects)):
            csv_header += ["obj{}_pos".format(j), "obj{}_type".format(j)]
        self.csv_writer.writerow(csv_header)

        self.world = world_objects  # keep a reference to the *Model structure
        self.objs = []
        for model in self.world:
            self.objs += [model.pos, model.type]


        self.PEDESTRIAN_TOPICS = Telemetry._get_ped_topics(no_peds)
        self.update_rate = rate
        
        self.drone_ref = DroneModel()
        self.subj_ref = SubjectModel()
        self.peds_ref = dict()
        for n in range(no_peds):
            # Pedestrians are numbered from 0 to (n-1)
            self.peds_ref[n] = PedestrianModel()
        
        self.daemon = TelemetryConsumer(
            "tkgndctrl",
            TelemetryConsumer.DEFAULT_CTRLADDR,
            TelemetryConsumer.DEFAULT_DATAPORT,
            self.update_rate,
            self._on_sample
        )

        self.daemon.start()
        # self.looper = self.daemon.looper

        # self.daemon_thread = threading.Thread(
        #     name="daemon",
        #     target=self.daemon.start,
        #     args=()
        # )
        # self.daemon_thread.setDaemon(True)

    @staticmethod
    def _get_ped_topics(n_peds):
        """
        Get the list of pedestrian topics
        :param n_peds:
        """
        ls = []
        for n in range(n_peds):
            ls += [coord.format(n) for coord in Telemetry.PED_TEMPL_TOPICS]
        return ls

    def stop(self):
        self.daemon.stop()
        self.file.close()

    def close_csv(self, sig, frame):
        self.file.close()

    def _on_drone_sample(self, ts, tname, tid, val):
        """
        Run an action when data from the drone is received from a callback
        :param ts: Timestamp
        :param tname: Topic name
        :param tid: Topic id (either X,Y,Z)
        :param val: Data value (float64)
        """
        # print("Drone: ", ts, tname, tid, val)

        if tid == "x":
            self.drone_ref.set_x(ts, val)
        elif tid == "y":
            self.drone_ref.set_y(ts, val)
        elif tid == "z":
            self.drone_ref.set_z(ts, val)

    def _on_pedestrian_sample(self, ts, pid, tname, tid, val):
        """
        Run an action when data from a pedestrian is received from a callback
        :param ts: Timestamp
        :param pid: Pedestrian id (from 0 to n)
        :param tname: Topic name
        :param tid: Topic id (either X,Y,Z)
        :param val: Data value (float64)
        """
        # print("Pedestrian: ", pid, ":", ts, tname, tid, val)
        # print(pid, type(pid))
        npid = int(pid)

        if tid == "x":
            self.peds_ref[npid].set_x(ts, val)
        elif tid == "y":
            self.peds_ref[npid].set_y(ts, val)
        elif tid == "z":
            self.peds_ref[npid].set_z(ts, val)

    def _on_subject_sample(self, ts, tname, tid, val):
        """
        Run an action when data from the subject is received from a callback
        :param ts: Timestamp
        :param tname: Topic name
        :param tid: Topic id (either X,Y,Z)
        :param val: Data value (float64)
        """
        # print("Subject: ", ts, tname, tid, val)

        if tid == "x":
            self.subj_ref.set_x(ts, val)
        elif tid == "y":
            self.subj_ref.set_y(ts, val)
        elif tid == "z":
            self.subj_ref.set_z(ts, val)

    def _on_sample(self, dname, tname, pid, ts, coord, data):
        """"""
        if dname in Telemetry.DRONE_TOPICS:
            self._on_drone_sample(ts, tname, coord, data)
        elif dname in Telemetry.SUBJECT_TOPICS:
            self._on_subject_sample(ts, tname, coord, data)
        elif dname in self.PEDESTRIAN_TOPICS:
            self._on_pedestrian_sample(ts, pid, tname, coord, data)

        # All written data should be complete i.e. no coord can be 'None'
        pub_rightly = self.drone_ref.complete() and \
            self.subj_ref.complete() and \
            all([model.complete() for (_, model) in self.peds_ref.items()])
        
        if pub_rightly:
            drone = self.drone_ref.get_pos()
            subjt = self.subj_ref.get_pos()
            peds = [model.get_pos() for (_, model) in self.peds_ref.items()]
            # get all data and write it. each row is 1s in the simulation
            # timestamp, drone.pos, subject.pos, pedestrian0.pos, ped.., obj1.pos, obj1.type, ..
            # each position is a tuple (x,y,z). Each type is str(ObjectClass).
            self.csv_writer.writerow([ts, drone, subjt] + peds + self.objs) 
    
    def get_drone(self):
        """Get the drone reference"""
        return self.drone_ref

    def get_subject(self):
        """Get the subject reference"""
        return self.subj_ref

    def get_pedestrian(self, pid):
        """Get the pedestrian reference by its pedestrian ID or None"""
        return self.peds_ref.get(pid)

    def set_drone_callback(self, fun):
        # NOT IMPLEMENTED
        if self.flags["drone"] and False:
            self.drone_callback = fun

    def set_peds_callback(self, fun):
        # NOT IMPLEMENTED
        if self.flags["peds"] and False:
            self.peds_callback = fun

    def set_subject_callback(self, fun):
        # NOT IMPLEMENTED
        if self.flags["subject"] and False:
            self.subject.callback = fun