#!/usr/bin/env python
# Implementation of a telemetryd consumer tool to create intermediate and written
# statistical data about drone and pedestrian poses

from __future__ import print_function
from time import sleep
from sim_objs.drone import DroneModel
from sim_objs.pedestrian import PedestrianModel

import os
import sys
import signal
import logging
import optparse
import Tkinter as tk

# Add parrot-specific libs
PARROT_COMMON = "/Documents/code/parrot-groundsdk/packages/common"
POMP_LIB = os.path.expanduser("~") + PARROT_COMMON + "/libpomp/python"
TELEMETRYD_LIB = os.path.expanduser("~") + PARROT_COMMON + "/telemetry/tools"

sys.path.append(POMP_LIB)
sys.path.append(TELEMETRYD_LIB)

# Import remaining modules
import pomp

from tlmb_parser import TlmbVarType, TlmbSection, TlmbSample

# ===== Telemetry constants =====
GNDCTRL_PROTOCOL_VERSION = 1
GNDCTRL_MSG_CONN_REQ =1
GNDCTRL_MSG_CONN_RESP =2
GNDCTRL_MSG_SUBSCRIBE_REQ = 3
GNDCTRL_MSG_SUBSCRIBE_RESP = 4
GNDCTRL_MSG_UNSUBSCRIBE_REQ = 5
GNDCTRL_MSG_UNSUBSCRIBE_RESP = 6
GNDCTRL_MSG_SECTION_ADDED = 7
GNDCTRL_MSG_SECTION_REMOVED = 8
GNDCTRL_MSG_SECTION_CHANGED = 9
GNDCTRL_MSG_SECTION_SAMPLE = 10

# Sampling constants
SAMPLE_RATE = 1000 * 1000    # Samples every 200ms
MSG_RATE = 1000 * 1000      # Message every 1s

# Script default arguments
DEFAULT_CTRLADDR = "inet:127.0.0.1:9060"
DEFAULT_DATAPORT = "5000"


#============== DRONE AND PEDESTRIAN TOPICS ===============
#==========================================================

DRONE_TOPICS = [
    "omniscient_bebop2.worldPosition.x",
    "omniscient_bebop2.worldPosition.y",
    "omniscient_bebop2.worldPosition.z",
    "omniscient_anafi.worldPosition.x",
    "omniscient_anafi.worldPosition.y",
    "omniscient_anafi.worldPosition.z"
]

PED_TEMPL_TOPICS = [
    "omniscient_pedestrian{}.worldPosition.x",
    "omniscient_pedestrian{}.worldPosition.y",
    "omniscient_pedestrian{}.worldPosition.z",
]

def get_ped_topics(n_peds):
    """
    Get the list of pedestrian topics
    :param n_peds:
    """
    ls = []
    for n in range(n_peds):
        ls += [coord.format(n) for coord in PED_TEMPL_TOPICS]
    return ls


#============== TELEMETRY CONSUMER CLASS ==================
#==========================================================

class TelemetryConsumer:
    def __init__(self, name, ctrlAddr, dataPort, n_peds):
        """
        :param name:
        :param ctrlAddr:
        :param dataPort:
        :param n_peds:
        """

        # Telemetryd-specific consumer options
        self.name = name
        self.ctrlAddr = ctrlAddr
        self.dataPort = dataPort
        self.ctrlCtx = pomp.Context(TelemetryConsumer._CtrlEventHandler(self))
        self.dataCtx = pomp.Context(TelemetryConsumer._DataEventHandler(self))
        self.sections = {}

        # Assign the available drone and pedestrian topics
        self.DRONE_TOPICS = DRONE_TOPICS
        self.PEDESTRIAN_TOPICS = get_ped_topics(n_peds)

        self.drone_ref = DroneModel()
        self.ped_ref = dict()
        for n in range(n_peds):
            self.ped_ref[n] = PedestrianModel()

        # Init console logging
        print("--------------------")
        print("Ts | SectionID | VarType | VarName | VarValue")
        print("--------------------")

    def _onDroneSample(self, ts, tname, tid, val):
        """
        Run an action when data from the drone is received from a callback
        :param ts: Timestamp
        :param tname: Topic name
        :param tid: Topic id (either X,Y,Z)
        :param val: Data value (float64)
        """
        if tid == "x":
            self.drone_ref.x = val
        elif tid == "y":
            self.drone_ref.y = val
        elif tid == "z":
            self.drone_ref.z = val

    def _onPedestrianSample(self, ts, pid, tname, tid, val):
        """
        Run an action when data from a pedestrian is received from a callback
        :param ts: Timestamp
        :param pid: Pedestrian id (from 0 to n)
        :param tname: Topic name
        :param tid: Topic id (either X,Y,Z)
        :param val: Data value (float64)
        """
        if tid == "x":
            self.ped_ref[pid].x = val
        elif tid == "y":
            self.ped_ref[pid].y = val
        elif tid == "z":
            self.ped_ref[pid].z = val        

    def recvSample(self, sectionId, timestamp, buf):
        """
        Processes a sample (either control or data)
        :param sectionId:
        :param timestamp:
        :param buf:
        """
        section = self.sections.get(sectionId, None)
        if section is None:
            return
        logging.debug("Sample: %s(%d) %d.%06d", section.sectionName, sectionId,
                timestamp[0], timestamp[1] // 1000)
        varOff = 0
        for varId in range(0, len(section.varDescs)):
            varDesc = section.varDescs[varId]
            varLen = varDesc.getTotalSize()
            if varOff + varLen > len(buf):
                break
            varBuf = buf[varOff:varOff+varLen]

            # dtype = TlmbVarType.toString(varDesc.varType)
            quantity = TlmbSample.extractQuantity(varDesc, varBuf)

            # =========================================
            # DATA FROM TELEMETRY CAN BE PROCESSED HERE
            # =========================================
            # print("Drone Data: {} {} {} {} {}".format(ts, topic_id, dtype, dname, val))
            dname = varDesc.getFullName()
            coord = dname.split(".")[-1]
            tname = dname[:-2]  # the whole name but '.x'
            data = float(quantity)
            ts = timestamp[1] // 1000

            if dname in self.DRONE_TOPICS:
                print("Drone:", ts, tname, coord, data)
                self._onDroneSample(ts, tname, coord, data)
            elif dname in self.PEDESTRIAN_TOPICS:
                pid = dname.split(".")[0][-1]  # gets the pedestrian ID
                print("Pedestrian", pid, ":", ts, tname, coord, data)
                self._onPedestrianSample(ts, int(pid), tname, coord, data)
            # =========================================

            varOff += varLen

    def start(self):
        """Starts the """
        (family, addr) = pomp.parseAddr(self.ctrlAddr)
        self.ctrlCtx.connect(family, addr)

        (family, addr) = pomp.parseAddr("inet:0.0.0.0:%u" % self.dataPort)
        self.dataCtx.bind(family, addr)

    def stop(self):
        """Stops the """
        self.ctrlCtx.stop()
        self.dataCtx.stop()

    def get_drone(self):
        """Get the drone reference"""
        return self.drone_ref

    def get_pedestrian(self, pid):
        """Get the pedestrian reference by its pedestrian ID or None"""
        return self.ped_ref.get(pid)

    class _CtrlEventHandler(pomp.EventHandler):
        def __init__(self, itf):
            """
            Creates a Control Event Handler
            :param itf:
            """
            self.itf = itf

        def onConnected(self, ctx, conn):
            """
            Run an action when the handler is connected.
            Here it sends a connection request.
            :param ctx pomp.Context:
            :param conn:
            """
            conn.send(
                GNDCTRL_MSG_CONN_REQ,
                "%u%s%u%u%u",
                GNDCTRL_PROTOCOL_VERSION,
                self.itf.name,
                self.itf.dataPort,
                SAMPLE_RATE,
                MSG_RATE
            )

        def onDisconnected(self, ctx, conn):
            """
            Run an action when the handler is disconnected.
            Here it clears the internal state.
            :param ctx pomp.Context:
            :param conn:
            """
            logging.info("Disconnected")
            self.sections = {}

        def recvMessage(self, ctx, conn, msg):
            """
            Run an action when the handler receives a message
            :param ctx pomp.Context:
            :param conn:
            :param msg:
            """
            if msg.msgid == GNDCTRL_MSG_CONN_RESP:
                dec = pomp.Decoder()
                dec.init(msg)
                status = dec.readU32()
                count = dec.readU32()
                logging.info("Connected: status=%d", status)
                for _ in range(0, count):
                    key = dec.readStr()
                    val = dec.readStr()
                    logging.info("%s='%s'", key, val)

            elif msg.msgid == GNDCTRL_MSG_SECTION_ADDED:
                (sectionId, sectionName) = msg.read("%u%s")
                section = TlmbSection(sectionId, sectionName)
                self.itf.sections[sectionId] = section
                logging.info("Section added: %s(%d)", sectionName, sectionId)

            elif msg.msgid == GNDCTRL_MSG_SECTION_REMOVED:
                (sectionId, ) = msg.read("%u")
                section = self.itf.sections.get(sectionId, None)
                if section is not None:
                    logging.info("Section removed: %s(%d)", section.sectionName, sectionId)
                    del self.itf.sections[sectionId]

            elif msg.msgid == GNDCTRL_MSG_SECTION_CHANGED:
                (sectionId, buf) = msg.read("%u%p")
                section = self.itf.sections.get(sectionId, None)
                if section is not None:
                    newSection = TlmbSection(sectionId, section.sectionName)
                    newSection.readHeader(buf)
                    logging.info("Section changed: %s(%d)", section.sectionName, sectionId)
                    self.itf.sections[sectionId] = newSection

            elif msg.msgid == GNDCTRL_MSG_SECTION_SAMPLE:
                # Only if client is configured to receive samples on the control channel
                (sectionId, sec, nsec, buf) = msg.read("%u%u%u%p")
                # self.itf.recvSample(sectionId, (sec, nsec), buf)

    class _DataEventHandler(pomp.EventHandler):
        def __init__(self, itf):
            """
            Creates a Data Event Handler
            :param itf:
            """
            self.itf = itf

        def onConnected(self, ctx, conn):
            """
            Run an action when the handler is connected
            :param ctx pomp.Context:
            :param conn:
            """
            pass

        def onDisconnected(self, ctx, conn):
            """
            Run an action when the handler is disconnected
            :param ctx pomp.Context:
            :param conn:
            """
            pass

        def recvMessage(self, ctx, conn, msg):
            """
            Run an action when the handler receives a message
            :param ctx pomp.Context:
            :param conn:
            :param msg:
            """
            if msg.msgid == GNDCTRL_MSG_SECTION_SAMPLE:
                (sectionId, sec, nsec, buf) = msg.read("%u%u%u%p")
                self.itf.recvSample(sectionId, (sec, nsec), buf)


#============== ALT 1: USE TKINTER MAIN LOOP ==============
#==========================================================
class App(tk.Frame):
    def __init__(self, master=None):
        """Creates a Tkinter app"""
        tk.Frame.__init__(self, master)

        # Prepare looper
        pomp.looper.prepareLoop()
        self.after(100, self._onIdle)

        self._panel = tk.Frame(self)
        self._panel.pack(expand=True, fill="both")
        self.pack(expand=True, fill="both")

        self.master.wm_geometry("155x15")
        self.master.wm_title("telemetry")
        # self.master.wm_withdraw()
        # self.master.wm_deiconify()

    def _onIdle(self):
        """Run an action if the app becomes idle"""
        pomp.looper.stepLoop()
        self.after(1000, self._onIdle)

    def destroy(self):
        """Run an action when the app is closed"""
        # Stop everything
        pomp.looper.exitLoop()

#============== ALT 2: SIMPLE SLEEP LOOP ==================
#==========================================================

def infinite_loop(step):
    """
    Runs an infinite loop where an action is executed every X ms
    :param step:
    """
    while True:
        pomp.looper.stepLoop()
        sleep(step)

#============== LOGGING AND PARSE UTILS ===================
#==========================================================

class DefaultOptions:
    def __init__(self):
        """Creates an object of default logging options"""
        self.quiet = False
        self.verbose = False
        self.log_level = 2

def parseArgs():
    """Setups the parser"""
    parser = optparse.OptionParser(usage=_USAGE)

    parser.add_option("-q", "--quiet",
        dest="quiet",
        action="store_true",
        default=False,
        help="be quiet")

    parser.add_option("-l", "--log-level",
        dest="log_level",
        default=2,
        help="log level (default is 2, highest is 1, lowest is 5)")

    parser.add_option("-v", "--verbose",
        dest="verbose",
        action="store_true",
        default=False,
        help="verbose output")

    # Parse arguments
    (options, args) = parser.parse_args()
    if len(args) != 2:
        print(
            "Bad number of arguments. Falling back to default arguments",
            "\nControl Address: {}".format(DEFAULT_CTRLADDR),
            "\nDataport: {}".format(DEFAULT_DATAPORT),
            file=sys.stderr
        )
        options = DefaultOptions()
        args = [DEFAULT_CTRLADDR, DEFAULT_DATAPORT]
    return (options, args)


_USAGE = (
    "usage: %prog [<options>] <ctrladdr> <dataport>\n"
    "Connect to a ishtar server\n"
    "\n"
    "  <options>: see below\n"
    "  <ctrladdr> : control address\n"
    "  <dataport> : data port\n"
    "\n"
    "<ctrladdr> format:\n"
    "  inet:<addr>:<port>\n"
    "  inet6:<addr>:<port>\n"
    "  unix:<path>\n"
    "  unix:@<name>\n"
)


def setupLog(options):
    """
    Setups the logger tool
    :param options:
    """
    lvl = {
        1: logging.DEBUG,
        2: logging.INFO,
        3: logging.WARNING,
        4: logging.ERROR,
        5: logging.CRITICAL
    }.get(options.log_level)

    logging.basicConfig(
        level=lvl,
        format="[%(levelname)s][%(asctime)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        stream=sys.stderr)

    logging.addLevelName(logging.CRITICAL, "C")
    logging.addLevelName(logging.ERROR, "E")
    logging.addLevelName(logging.WARNING, "W")
    logging.addLevelName(logging.INFO, "I")
    logging.addLevelName(logging.DEBUG, "D")

    # Setup log level
    if options.quiet == True:
        logging.getLogger().setLevel(logging.CRITICAL)
    elif options.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

#============== MAIN SCRIPT FUNCTION ======================
#==========================================================

if __name__ == "__main__":
    """Enables the telemetry logging and processing"""
    (options, args) = parseArgs()
    ctrl_addr, data_port = args[0], int(args[1])
    setupLog(options)

    # Runs the loop using the Tkinter loop routine
    try:
        root = tk.Tk()
        app = App(master=root)
        itf = TelemetryConsumer("tkgndctrl", ctrl_addr, data_port, 2)

        itf.start()
        app.mainloop()
        itf.stop()
    except KeyboardInterrupt:
        root.quit()
        root.destroy()
    sys.exit(0)

    # Runs the loop using simple 'sleep' routines
    # try:
    #     cons = TelemetryConsumer("illokdise", ctrl_addr, data_port)
        
    #     cons.start()
    #     infinite_loop(step=0.5)
    #     cons.stop()
    # except KeyboardInterrupt:
    #     pomp.looper.exitLoop()
    # sys.exit(0)
