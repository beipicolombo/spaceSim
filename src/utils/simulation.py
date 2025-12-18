
import numpy as np
import ephem
from src.interfaces.eventsAndTmTcDatabase import TC_DATABASE
from src.interfaces.eventsAndTmTcDatabase import EVT_DATABASE


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# DATA CLASSES
# --------------------------------------------------
# ...


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Simulation():
    def __init__(self):
        # TBW: add simParam as parameter
        self.param = Param()
        self.state  = State()
        # Interface
        self.interface = {}
        self.interface.update({"elapsedTime": self.state.elapsedTime})
        self.interface.update({"receivedTcList": []})

    def set(self):
        self.state.waitingTc = self.param.tcTimeline # Initialize waiting TC list

    def run(self):
        # Compute elapsed time
        Ts = self.param.Ts
        self.state.elapsedTime = self.state.elapsedTime + Ts
        # Retrieve received TCs
        receivedTcList = self.listenTc()
        # Update interface
        self.interface["elapsedTime"] = self.state.elapsedTime
        self.interface["receivedTcList"] = receivedTcList

    # TC listener
    def listenTc(self):
        # Get the elapsed time and TC timeline
        elapsedTime = self.state.elapsedTime
        # Initialization
        receivedTcList = []
        # If any TC is defined in the timeline
        if (len(self.state.waitingTc) > 0):
            # Check if the TC is received
            # TBW: handle if TCs are received at the same time
            if (elapsedTime >= self.state.waitingTc[0].time):
                receivedTcList.append(self.state.waitingTc.pop(0))

        # Update interface
        return receivedTcList

        
class State():
    def __init__(self):
        self.elapsedTime = 0
        self.waitingTc = []


# To be implemented with dataclass
class Param:
    # To be moved as higher level class
    def __init__(self):
        self.caseName = "Default"
        self.dateTimeStart = ephem.Date("2024/3/9 5:10:10")
        self.Ts = 1 # [s]
        self.Tend = 10*60 # [s]
        self.simOptions = SimOptions()        
        self.runOptions = RunOptions()
        self.ephemEpoch = ephem.J2000
        # Depends on other parameters
        self.timeVec = np.arange(0, self.Tend+self.Ts, self.Ts)
        self.nbPts = len(self.timeVec)
        # TC 
        self.tcTimeline = []
        self.tcDatabase = TC_DATABASE
        # Events
        self.eventsDatabase = EVT_DATABASE

    def updateTimeVec(self):
        self.timeVec = np.arange(0, self.Tend+self.Ts, self.Ts)
        self.nbPts = len(self.timeVec)


# Simulation options => to be verified if still needed
class SimOptions:
    def __init__(self):
        self.isGGTorqueEnabled = True
        self.isCtrlTorqueEnabled = True
        self.isRwAct = False
        self.isThrAct = True
        self.swInitAttitudeFrame = "J"
        self.massModelName = "Default"
                

# Run options       
class RunOptions:
    def __init__(self):
        self.swSetReferenceData = False
        self.swPlot = True
        self.swVisualPy = True
        self.visualPyRate = 300
        self.swExportData = True

        self.signalPathsToLog = []
        self.signalPathsToExport = []




