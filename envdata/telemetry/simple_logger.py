import sys
import logging
import os
import optparse
import signal
import threading

PARROT_COMMON = "/Documents/parrot/groundsdk/packages/common"

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

SAMPLE_RATE = 200 * 1000    # Samples every 200ms
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
    def __init__(self, app, name, ctrlAddr, dataPort):
        self.app = app
        self.name = name
        self.ctrlAddr = ctrlAddr
        self.dataPort = dataPort
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
            count = dec.readU32()
            logging.info("Connected: status=%d", status)
            for _ in range(0, count):
                key = dec.readStr()
                val = dec.readStr()
                logging.info("%s='%s'", key, val)
        elif msg.msgid == GNDCTRL_MSG_SECTION_ADDED:
            (sectionId, sectionName) = msg.read("%u%s")
            section = TlmbSection(sectionId, sectionName)
            self.sections[sectionId] = section
            logging.info("Section added: %s(%d)", sectionName, sectionId)
        elif msg.msgid == GNDCTRL_MSG_SECTION_REMOVED:
            (sectionId, ) = msg.read("%u")
            section = self.sections.get(sectionId, None)
            if section is not None:
                logging.info("Section removed: %s(%d)", section.sectionName, sectionId)
                self.app.sectionRemoved(section.sectionName)
                del self.sections[sectionId]
        elif msg.msgid == GNDCTRL_MSG_SECTION_CHANGED:
            (sectionId, buf) = msg.read("%u%p")
            section = self.sections.get(sectionId, None)
            if section is not None:
                newSection = TlmbSection(sectionId, section.sectionName)
                newSection.readHeader(buf)
                logging.info("Section changed: %s(%d)", section.sectionName, sectionId)
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
        logging.debug("Sample: %s(%d) %d.%06d", section.sectionName, sectionId,
                timestamp[0], timestamp[1] // 1000)
        varOff = 0
        for varId in range(0, len(section.varDescs)):
            varDesc = section.varDescs[varId]
            varLen = varDesc.getTotalSize()
            if varOff + varLen > len(buf):
                break
            varBuf = buf[varOff:varOff+varLen]
            self.app.sample(sectionId, timestamp, varId, varDesc, varBuf)
            varOff += varLen

    class _CtrlEventHandler(pomp.EventHandler):
        def __init__(self, itf):
            self.itf = itf
        def onConnected(self, ctx, conn):
            # Send connection request
            conn.send(GNDCTRL_MSG_CONN_REQ, "%u%s%u%u%u",
                     GNDCTRL_PROTOCOL_VERSION, self.itf.name, self.itf.dataPort,
                     SAMPLE_RATE, MSG_RATE)
        def onDisconnected(self, ctx, conn):
            # Clear internal state
            logging.info("Disconnected")
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
    def __init__(self, args):
        self.sock_family = None
        self.sock_addr = None
        self.running = False
        self.thread = None
        self.itf = GndCtrlItf(self, "example", args[0], int(args[1]))
        signal.signal(signal.SIGINT,
                      lambda signal, frame: self._signal_handler())
        signal.signal(signal.SIGTERM,
                      lambda signal, frame: self._signal_handler())

    def _signal_handler(self):
        self.running = False

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

    def sample(self, sectionId, timestamp, varId, varDesc, buf):
        print(sectionId, timestamp, varId, varDesc, buf)

#===============================================================================
#===============================================================================
def parseArgs():
    # Setup parser
    parser = optparse.OptionParser(usage=_USAGE)

    parser.add_option("-q", "--quiet",
        dest="quiet",
        action="store_true",
        default=False,
        help="be quiet")

    parser.add_option("-v", "--verbose",
        dest="verbose",
        action="store_true",
        default=False,
        help="verbose output")

    # Parse arguments
    (options, args) = parser.parse_args()
    if len(args) != 2:
        parser.error("Bad number or arguments")
    return (options, args)

#===============================================================================
#===============================================================================
class DefOpts:
    def __init__(self):
        self.quiet = False
        self.verbose = False


def setupLog(options):
    logging.basicConfig(
        level=logging.INFO,
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

#===============================================================================
#===============================================================================

# This script is intended to be ran alone
# Uncomment this to try it
# if __name__ == "__main__":
#     # (options, args) = parseArgs()
#     args = ["inet:127.0.0.1:9060", 5000]
#     setupLog(DefOpts())

#     try:
#         app = App(args)
#         app.start()
#     except KeyboardInterrupt:
#         root.destroy()
#     sys.exit(0)
