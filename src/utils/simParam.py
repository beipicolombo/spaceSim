# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np
import ephem


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
        self.swInitAttitudeFrame = "I"
        self.massModelName = "Default"
                
# Run options       
class RunOptions:
    def __init__(self):
        self.isReferenceData = False
        self.isPlot = True
        self.swVisualPy = True
        self.swVisualPyLabelsAocsMode = True
        self.swVisualPyLabelsAttitude = False
        self.visualPyRate = 100
        self.saveData = False

        self.signalPathsToLog = []
        self.signalPathsToExport = []




