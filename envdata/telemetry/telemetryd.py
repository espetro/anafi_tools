from time import sleep
from datetime import datetime

import sys, os
import logging
import optparse
import signal
import threading

# PLEASE PROVIDE THE PARROT GROUNDSDK COMMON DIRECTORY
PARROT_COMMON = "/Documents/parrot/groundsdk/packages/common"
# PARROT_COMMON = "/Documents/code/parrot-groundsdk/packages/common"


#===============================================================================
#===============================================================================
POMP_LIB = os.path.expanduser("~") + PARROT_COMMON + "/libpomp/python"
TELEMETRYD_LIB = os.path.expanduser("~") + PARROT_COMMON + "/telemetry/tools"

sys.path.append(POMP_LIB)
sys.path.append(TELEMETRYD_LIB)

import pomp

from tlmb_parser import TlmbSection, TlmbSample

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

SAMPLE_RATE = 1000 * 1000    # Samples every 200ms
MSG_RATE = 1000 * 1000      # Message every 1s

#===============================================================================
#===============================================================================
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

#===============================================================================
#===============================================================================
class GndCtrlItf(object):
    """Creates a TCP connection interface with the telemetry daemon"""
    def __init__(self, app, name, ctrlAddr, dataPort, rate):
        self.app = app
        self.name = name
        self.ctrlAddr = ctrlAddr
        self.dataPort = dataPort
        self.rate = rate

        self.ctrlCtx = pomp.Context(GndCtrlItf._CtrlEventHandler(self))
        self.dataCtx = pomp.Context(GndCtrlItf._DataEventHandler(self))
        self.sections = {}

    def start(self):
        (family, addr) = pomp.parseAddr(self.ctrlAddr)
        self.ctrlCtx.connect(family, addr)
        (family, addr) = pomp.parseAddr("inet:0.0.0.0:%u" % self.dataPort)
        self.dataCtx.bind(family, addr)

    def stop(self):
        self.ctrlCtx.stop()
        self.dataCtx.stop()

    def recvCtrlMsg(self, msg):
        if msg.msgid == GNDCTRL_MSG_CONN_RESP:
            dec = pomp.Decoder()
            dec.init(msg)
            status = dec.readU32()
            print("Connected: status={}".format(status))
            
        elif msg.msgid == GNDCTRL_MSG_SECTION_ADDED:
            (sectionId, sectionName) = msg.read("%u%s")
            section = TlmbSection(sectionId, sectionName)
            self.sections[sectionId] = section

        elif msg.msgid == GNDCTRL_MSG_SECTION_REMOVED:
            (sectionId, ) = msg.read("%u")
            section = self.sections.get(sectionId, None)
            if section is not None:
                self.app.sectionRemoved(section.sectionName)
                del self.sections[sectionId]

        elif msg.msgid == GNDCTRL_MSG_SECTION_CHANGED:
            (sectionId, buf) = msg.read("%u%p")
            section = self.sections.get(sectionId, None)
            if section is not None:
                newSection = TlmbSection(sectionId, section.sectionName)
                newSection.readHeader(buf)
                self.sections[sectionId] = newSection
                
        elif msg.msgid == GNDCTRL_MSG_SECTION_SAMPLE:
            # Only if client is configured to receive samples on the control channel
            (sectionId, sec, nsec, buf) = msg.read("%u%u%u%p")
            self.recvSample(sectionId, (sec, nsec), buf)

    def recvDataMsg(self, msg):
        if msg.msgid == GNDCTRL_MSG_SECTION_SAMPLE:
            (sectionId, sec, nsec, buf) = msg.read("%u%u%u%p")
            self.recvSample(sectionId, (sec, nsec), buf)

    def recvSample(self, sectionId, timestamp, buf):
        section = self.sections.get(sectionId, None)
        if section is None:
            return

        varOff = 0
        for varId in range(0, len(section.varDescs)):
            varDesc = section.varDescs[varId]
            varLen = varDesc.getTotalSize()
            if varOff + varLen > len(buf):
                break
            varBuf = buf[varOff:varOff+varLen]

            quantity = TlmbSample.extractQuantity(varDesc, varBuf)
            full_name = varDesc.getFullName()
            spaces = full_name.split(".")

            data = {
                "ts": timestamp[0],
                "topic": full_name,
                "namespace": spaces[0],
                "coord": spaces[-1],
                "value": float(quantity),
                "pid": spaces[0][-1]  # for peds
            }
            self.app.sample(data)

            varOff += varLen

    class _CtrlEventHandler(pomp.EventHandler):
        def __init__(self, itf):
            self.itf = itf
        def onConnected(self, ctx, conn):
            # Send connection request
            conn.send(GNDCTRL_MSG_CONN_REQ, "%u%s%u%u%u",
                     GNDCTRL_PROTOCOL_VERSION, self.itf.name, self.itf.dataPort,
                     self.itf.rate, self.itf.rate)
        def onDisconnected(self, ctx, conn):
            # Clear internal state
            print("Disconnected")
            self.sections = {}
        def recvMessage(self, ctx, conn, msg):
            self.itf.recvCtrlMsg(msg)

    class _DataEventHandler(pomp.EventHandler):
        def __init__(self, itf):
            self.itf = itf
        def onConnected(self, ctx, conn):
            pass
        def onDisconnected(self, ctx, conn):
            pass
        def recvMessage(self, ctx, conn, msg):
            self.itf.recvDataMsg(msg)


class App():
    """Wraps tkgndcntrl in a separate Thread"""
    def __init__(self, proc_name, sample_fun, rate=1000, ctrladdr="inet:127.0.0.1:9060", dataport="5000"):
        self.sock_family = None
        self.sock_addr = None
        self.running = False
        self.thread = None
        
        self.sample = sample_fun

        self.itf = GndCtrlItf(self, proc_name, ctrladdr, int(dataport), rate)

        # signal.signal(signal.SIGINT,
        #               lambda signal, frame: self._signal_handler())
        # signal.signal(signal.SIGTERM,
        #               lambda signal, frame: self._signal_handler())

    def _signal_handler(self):
        print("Signal handler")
        self.stop()

    def __del__(self):
        if self.running:
            self.stop()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.worker)
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def worker(self):
        # setup loop for main thread
        pomp.looper.prepareLoop()

        # create pomp context
        self.itf.start()

        # run main loop
        while self.running:
            pomp.looper.stepLoop(maxMsg=1, timeout=1)

        # destroy pomp context
        self.itf.stop()

    def sectionRemoved(self, sectionName):
        pass

#===============================================================================
#===============================================================================

class SampleFun:
    def __init__(self, fname):
        self.ffile = open(fname, "w+")
        self.timestamp = None

    def sample(self, data):            
        self.timestamp = data["ts"]

        if not self.ffile.closed:
            print(self.timestamp)
            
            _str = "{} {} {} {}\n".format(
                data["topic"],
                data["namespace"],
                data["coord"],
                data["value"]
            )
            self.ffile.write(_str)


if __name__ == "__main__":
    fdir = "/home/pachacho/Documents/anafi_tools/data/train"
    # (options, args) = parseArgs()
    # setupLog(options)
    # setupLog(DefaultOpts())

    args = [
        "inet:127.0.0.1:9060",
        5000
    ]

    for i in range(5):
        print("Run no {}".format(i))
        proc_name = "run{}".format(i)

        fname = "{}/example{}.log".format(
            fdir,
            datetime.now().strftime("%y%m%d_%H%M%S")
        )

        obj = SampleFun(fname)

        try:
            app = App(proc_name, obj.sample, 1000, args[0], args[1])
            app.start()
            print("Sleeping 10s (like running other tasks e.g. joining teleop)")
            sleep(10)
        except KeyboardInterrupt:
            app.stop()
        finally:
            app.stop()

        print("ts: {}".format(obj.timestamp))
        obj.ffile.close()

    sys.exit(0)
