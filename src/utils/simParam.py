# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np
import ephem
import src.interfaces.eventsAndTmTc as eventsAndTmTc
import src.interfaces.eventsAndTmTcDatabase as eventsAndTmTcDatabase



# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# Simulation parameters and options
# --------------------------------------------------
# Simulation parameters
class SimParam:
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
        self.tcDatabase = eventsAndTmTcDatabase.tcDatabase
        # Events
        self.eventsDatabase = eventsAndTmTcDatabase.eventDatabase

    def updateTimeVec(self):
        self.timeVec = np.arange(0, self.Tend+self.Ts, self.Ts)
        self.nbPts = len(self.timeVec)

# Simulation options
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




