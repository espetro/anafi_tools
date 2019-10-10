# -*- coding: utf-8 -*-

from __future__ import print_function
from drone_model import DroneModel
from subject_model import SubjectModel
from pedestrian_model import PedestrianModel
from telemetryd import TelemetryDaemon

import csv
import signal
import threading

from random import randint

PED_TEMPL_TOPICS = [
    "omniscient_pedestrian{}.worldPosition.x", "omniscient_pedestrian{}.worldPosition.y",
    "omniscient_pedestrian{}.worldPosition.z",
]

def get_ped_topics(n_peds):
    """Get the list of pedestrian topics"""
    ls = []
    for n in range(n_peds):
        ls += [coord.format(n) for coord in PED_TEMPL_TOPICS]
    return ls


class TelemetryBag:

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
            "ts": self.drone.x[0],  # get the drone timestamp
            "drone": self.drone,
            "subject": self.subject,
            "peds": self.peds,
            "objs": self.objs
        }

    def print_data(self):
        data = self.get_data()
        for (k,v) in data.items():
            print(k, v)


class TelemetryConsumer:
    DRONE_TOPICS = [
        "omniscient_bebop2.worldPosition.x", "omniscient_bebop2.worldPosition.y",
        "omniscient_bebop2.worldPosition.z", "omniscient_anafi.worldPosition.x",
        "omniscient_anafi.worldPosition.y", "omniscient_anafi.worldPosition.z"
    ]

    SUBJECT_TOPICS = [
        "omniscient_subject.worldPosition.x", "omniscient_subject.worldPosition.y",
        "omniscient_subject.worldPosition.z"
    ]

    def __init__(self, processing_fun, no_peds=0, objs=None):

        self.PEDESTRIAN_TOPICS = get_ped_topics(no_peds)

        self.fun = processing_fun
        self.bag = TelemetryBag(no_peds, objs)

        self.daemon = TelemetryDaemon(on_sample=self.store_in_bag)  
        
    def start(self):
        self.daemon.start()

    def stop(self):
        self.daemon.stop()
        
    def store_in_bag(self, data):
        """"""
        
        if randint(0,100) == 99:
            print(data)

        # self.bag.print_data()

        if data["topic"] in TelemetryConsumer.DRONE_TOPICS:
            self.bag.drone.set_val(data["ts"], data["coord"], data["value"])
        elif data["topic"] in TelemetryConsumer.SUBJECT_TOPICS:
            self.bag.subject.set_val(data["ts"], data["coord"], data["value"])
        elif data["topic"] in self.PEDESTRIAN_TOPICS:
            self.bag.peds[data["pid"]].set_val(data["ts"], data["coord"], data["value"])

        if self.bag.is_full():
            # All written data should be complete i.e. no coord can be 'None'
            # Also the timestamps have to match
        
            # self.bag.print_data()

            data = self.bag.get_data()
            self.fun(data)
