from __future__ import print_function

import sys

# Add libpomp and telemetry parsing tools
sys.path.append("{}/{}/libpomp/python".format(HOME, PARROT_COMMON))
sys.path.append("{}/{}/telemetry/tools".format(HOME, PARROT_COMMON))

import os
import pomp
from tlmb_parser import TlmbVarType, TlmbSection, TlmbSample

HOME = os.path.expanduser("~")
PARROT_COMMON = "code/parrot-groundsdk/packages/common"
TCP_PORT = 9800

# =================================



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
        def recvMessage(self, ctx, conn, msg):
            self.itf.recvDataMsg(msg)

#===============================================================================
#===============================================================================
class App(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)

        # Prepare looper
        pomp.looper.prepareLoop()
        self.after(100, self._onIdle)

        self._panel = tk.Frame(self)
        self._vbar = tk.Scrollbar(self._panel, orient="vertical")
        self._hbar = tk.Scrollbar(self._panel, orient="horizontal")

        self._treeView = ttk.Treeview(self._panel,
                columns=["Timestamp", "Type", "Value"],
                displaycolumns="#all",
                show="tree headings",
                selectmode="browse",
                yscrollcommand=self._vbar.set,
                xscrollcommand=self._hbar.set)

        self._treeView.heading("#0", text="Name", anchor="w")
        self._treeView.heading("Timestamp", text="Timestamp", anchor="w")
        self._treeView.heading("Type", text="Type", anchor="w")
        self._treeView.heading("Value", text="Value", anchor="w")
        self._treeView.column("Timestamp", minwidth=125, width=125, stretch=False)
        self._treeView.column("Type", minwidth=75, width=75, stretch=False)
        self._treeView.column("Value", minwidth=300, width=300)

        self._vbar["command"] = self._treeView.yview
        self._hbar["command"] = self._treeView.xview

        self._vbar.pack(side="right", fill="y")
        self._hbar.pack(side="bottom", fill="x")
        self._treeView.pack(side="left", expand=True, fill="both")
        self._panel.pack(expand=True, fill="both")
        self.pack(expand=True, fill="both")

        self.master.wm_geometry("800x500")
        self.master.wm_title("tkgndctrl")

    def _onIdle(self):
        pomp.looper.stepLoop()
        self.after(100, self._onIdle)

    def _clearTreeView(self):
        self._treeView.delete()

    def _getItemId(self, sectionId, varId):
        return sectionId * 1000 + varId + 1

    def _findItem(self, parent, name):
        for item in self._treeView.get_children(parent):
            if self._treeView.item(item, option="text") == name:
                return item
        return None

    def _addVar(self, sectionId, varId, varDesc):
        # Slip name at '.'
        fields = varDesc.getFullName().split(".")

        # Construct tree
        parent = ""
        for field in fields[:-1]:
            item = self._findItem(parent, field)
            if item is None:
                item = self._treeView.insert(parent, "end", text=field)
            parent = item

        # Add item with last component of name
        self._treeView.insert(parent, "end",
                text=fields[-1],
                iid=self._getItemId(sectionId, varId))

    def _updateVar(self, sectionId, varId, varDesc, timestamp, data):
        values = [
            "%u.%06u" % (timestamp[0], timestamp[1] // 1000),
            TlmbVarType.toString(varDesc.varType),
            data
        ]
        self._treeView.item(self._getItemId(sectionId, varId), values=values)

    def sectionRemoved(self, sectionName):
        for item in self._treeView.get_children(""):
            if self._treeView.item(item, option="text") == sectionName:
                self._treeView.delete(item)
                return

    def sample(self, sectionId, timestamp, varId, varDesc, buf):
        if not self._treeView.exists(self._getItemId(sectionId, varId)):
            self._addVar(sectionId, varId, varDesc)
        quantity = TlmbSample.extractQuantity(varDesc, buf)
        data = repr(quantity)
        self._updateVar(sectionId, varId, varDesc, timestamp, data)

    def destroy(self):
        # Stop everything
        pomp.looper.exitLoop()

#===============================================================================
#===============================================================================
def main():
    (options, args) = parseArgs()
    setupLog(options)

    try:
        root = tk.Tk()
        app = App(master=root)
        itf = GndCtrlItf(app, "tkgndctrl", args[0], int(args[1]))
        itf.start()
        app.mainloop()
        itf.stop()
        root.destroy()
    except KeyboardInterrupt:
    sys.exit(0)

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
if __name__ == "__main__":
    main()
