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
    def __init__(self, Ts, Tend, isGGTorqueEnabled = True, isCtrlTorqueEnabled = True, isPlot = True):
        self.Ts = Ts
        self.Tend = Tend
        self.timeVec = np.arange(0, Tend+Ts, Ts)
        self.nbPts = len(self.timeVec)
        self.simOptions = SimOptions(isGGTorqueEnabled = True, isCtrlTorqueEnabled = True)        
        self.runOptions = RunOptions(isPlot)
        self.ephemEpoch = ephem.J2000

# Simulation options
class SimOptions:
    def __init__(self, isGGTorqueEnabled = True, isCtrlTorqueEnabled = True):
        self.isGGTorqueEnabled = isGGTorqueEnabled
        self.isCtrlTorqueEnabled = isCtrlTorqueEnabled
        self.isRwAct = False
        self.isThrAct = True

# Run options       
class RunOptions:
    def __init__(self, isPlot = False):
        self.isPlot = isPlot
