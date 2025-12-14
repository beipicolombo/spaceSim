# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np
import ephem
import src.interfaces.eventsAndTmTc as eventsAndTmTc
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
        
    def run(self):
        # Compute elapsed time
        Ts = self.param.Ts
        self.state.elapsedTime = self.state.elapsedTime + Ts
        # Update interface
        self.interface["elapsedTime"] = self.state.elapsedTime
        
        
class State():
    def __init__(self):
        self.elapsedTime = 0


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




