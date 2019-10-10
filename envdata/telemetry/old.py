#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import sys
import time
import signal
import logging
import optparse
import threading

# Add parrot-specific libs
PARROT_COMMON = "/Documents/parrot/groundsdk/packages/common"
# PARROT_COMMON = "/Documents/code/parrot-groundsdk/packages/common"
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


#============== TELEMETRY DAEMON CLASS ====================
#==========================================================

class SimpleDmn(threading.Thread):
    def __init__(self):
        """Setups the pomp service in a separate thread"""
        threading.Thread.__init__(self)
        # pomp.looper.prepareLoop()

    def start(self):
        self.running = True
        while self.running:
            k = 5 * 4
            # print(1)
            # pomp.looper.stepLoop(maxMsg=1, timeout=1)
    
    def stop(self):
        print("Exiting the damned loop")
        # sys.exit(0)
        self.running = False
        # pomp.looper.exitLoop()

class SimpleItf:
    """A simple object to try pomp.looper in the bg (threading vs multiprocessing)"""
    def __init__(self, range=1e4):
        self.range = 1e4
        self.task = SimpleDmn()
        self.task.setDaemon(True)
        
        self.go_on = True

        signal.signal(signal.SIGINT, self.stop)
        pomp.looper.prepareLoop()

    def start(self):
        self.task.start()
        i = 0
        while i < self.range and self.go_on:
            time.sleep(1)
            print("Illo")
            i += 1

    def stop(self, sig, frame):
        print("Exiting task")
        # self.task.exit()
        print("Exiting main app")
        self.go_on = False
        # sys.exit(0)


class TelemetryDaemon:
    SAMPLE_RATE = 1000 * 1000    # Samples every 1000ms
    MSG_RATE = 1000 * 1000      # Message every 1s
    DEFAULT_CTRLADDR = "inet:127.0.0.1:9060"
    DEFAULT_DATAPORT = "5000"

    def __init__(self, on_sample, name="tkgndctrl",
        ctrlAddr="inet:127.0.0.1:9060", dataPort=5000, rate=1000):
        """
        Setups the telemetry daemon's default configuration. Rate defaults to
        1000 ms (1s).
        """
        self.name = name
        self.sample_rate = int(rate) * 1000
        self.ctrlAddr = ctrlAddr
        self.dataPort = dataPort

        self.on_sample = on_sample
        self.ctrlCtx = pomp.Context(TelemetryDaemon._CtrlEventHandler(self))
        self.dataCtx = pomp.Context(TelemetryDaemon._DataEventHandler(self))

        self.looper = TelemetryDaemon.PompThread()

        self.sections = {}

        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        print("\nExiting telemetry daemon loop!\n")
        sys.exit(0)
        # self.stop()

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

            # tlmb-dependent data pre-processing
            # dtype = TlmbVarType.toString(varDesc.varType)
            quantity = TlmbSample.extractQuantity(varDesc, varBuf)

            full_name = varDesc.getFullName()
            spaces = full_name.split(".")
            # related to "omniscient_bebop2.worldPosition.x"
            data = {
                "ts": timestamp[0],
                "topic": full_name,
                "namespace": spaces[0],
                "coord": spaces[-1],
                "value": float(quantity),
                "pid": spaces[0][-1]  # for pedestrians
            }
            
            # =========================================
            # DATA FROM TELEMETRY CAN BE PROCESSED HERE
            # =========================================
            self.on_sample(data)
            # =========================================

            varOff += varLen

    def start(self):
        """Starts the connection with the telemetry service and the pomp loop"""
        (family, addr) = pomp.parseAddr(self.ctrlAddr)
        self.ctrlCtx.connect(family, addr)

        (family, addr) = pomp.parseAddr("inet:0.0.0.0:%u" % self.dataPort)
        self.dataCtx.bind(family, addr)

        self.looper.start()

    def stop(self):
        """Stops the connection with the telemetry service and the pomp loop"""
        self.ctrlCtx.stop()
        self.dataCtx.stop()
        print("Telemetry connections closed. Go now with the loop")
        self.looper.stop()

    class PompThread(threading.Thread):
        def __init__(self):
            """Setups the pomp service in a separate thread"""
            threading.Thread.__init__(self)
            pomp.looper.prepareLoop()

        def start(self):
            self.running = True
            while self.running:
                pomp.looper.stepLoop(maxMsg=1, timeout=1)
        
        def stop(self):
            print("Exiting the damned loop")
            sys.exit(0)
            self.running = False
            pomp.looper.exitLoop()

    class _CtrlEventHandler(pomp.EventHandler):
        def __init__(self, interface):
            """
            Creates a Control Event Handler
            """
            self.itf = interface

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
                self.itf.sample_rate, #SAMPLE_RATE
                self.itf.sample_rate  #MSG_RATE
            )

        def onDisconnected(self, ctx, conn):
            """
            Run an action when the handler is disconnected.
            Here it clears the internal state.
            :param ctx pomp.Context:
            :param conn:
            """
            print("Disconnected")
            logging.info("Disconnected")
            self.sections = {}
            # sys.exit(0)

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
        def __init__(self, interface):
            self.itf = interface

        def onConnected(self, ctx, conn):
            pass

        def onDisconnected(self, ctx, conn):
            pass

        def recvMessage(self, ctx, conn, msg):
            if msg.msgid == GNDCTRL_MSG_SECTION_SAMPLE:
                (sectionId, sec, nsec, buf) = msg.read("%u%u%u%p")
                self.itf.recvSample(sectionId, (sec, nsec), buf)


# =========== RUN AS SCRIPT ===========
# =====================================
_USAGE = (
    "usage: %prog <num_peds> <world>\n"
    "Connect to a ishtar server\n"
    "\n"
    "  <num_peds> : Number of pedestrians in the simulation\n"
    "  <world> : World absolute file path\n"
    "\n"
)

def parseArgs():
    # Setup parser
    parser = optparse.OptionParser(usage=_USAGE)

    # parser.add_option("-q", "--quiet",
    #     dest="quiet",
    #     action="store_true",
    #     default=False,
    #     help="be quiet")

    # parser.add_option("-v", "--verbose",
    #     dest="verbose",
    #     action="store_true",
    #     default=False,
    #     help="verbose output")

    # Parse arguments
    (_, args) = parser.parse_args()
    if len(args) != 2:
        parser.error("Bad number or arguments")
    return (None, args)


if __name__ == "__main__":
    # sample_fun defaults to 'print'
    # python telemetryd.py --no-peds 1 --world hello
    (_, args) = parseArgs()

    try:
        telem = TelemetryDaemon(on_sample=print)
        telem.start()

    except KeyboardInterrupt:
        sys.exit(0)
        telem.exit()
     