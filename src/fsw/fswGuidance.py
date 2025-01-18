# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np

deg2rad = np.pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------

class FswGuidanceParam:
    # To be moved as higher level class
    def __init__(self):
        self.Dummy = 0
        self.MODE = FswGuidanceMode()
        self.GUIDMODE_ATT_INERT_eulerAngGuid_BI = np.array([0, 0, 0])*deg2rad

class FswGuidanceMode:
    def __init__(self, fswGuidanceMode = "GUIDMODE_OFF"):
        self.MODE = fswGuidanceMode


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

def rateDampingGuidance():
    angRateGuid_B = np.array([0, 0, 0])
    eulerAngGuid_BI = np.array([0, 0, 0])
    return (angRateGuid_B, eulerAngGuid_BI)

def attitudeInertialGuidance(fswGuidanceParam):
    angRateGuid_B = np.array([0, 0, 0])
    eulerAngGuid_BI = fswGuidanceParam.GUIDMODE_ATT_INERT_eulerAngGuid_BI
    
    return (angRateGuid_B, eulerAngGuid_BI)

def offGuidance():
    angRateGuid_B = np.array([0, 0, 0])
    eulerAngGuid_BI = np.array([0, 0, 0])
    
    return (angRateGuid_B, eulerAngGuid_BI)

def nadirGuidance(dcmIL):
    angRateGuid_B = np.array([0, 0, 0])
    eulerAngGuid_BI = np.array([0, 0, 0])
    
    return (angRateGuid_B, eulerAngGuid_BI)

def computeGuidance(fswGuidanceParam, lvlhFrame, modelsBus, fswBus):
    # Retrieve useful inputs
    zL_B = modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_B"].value
    zL_I = modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_I"].value
    
    # Initialize output bus
    fswGuidBusOut = fswBus.subBuses["guidance"]

    # Compute guidance angular rates and euler angles depending on the current guidance mode
    if (fswGuidanceParam.MODE == "GUIDMODE_ATT_INERT"):
        # Inertial attitude guidance mode
        (angRateGuid_B, eulerAngGuid_BI) = attitudeInertialGuidance(fswGuidanceParam)
    elif (fswGuidanceParam.MODE == "GUIDMODE_RATE_DAMPING"):
        # Rate damping guidance mode
        (angRateGuid_B, eulerAngGuid_BI) = rateDampingGuidance()
    elif (fswGuidanceParam.MODE == "GUIDMODE_ATT_NADIR"):
        # Nadir guidance mode
        (angRateGuid_B, eulerAngGuid_BI) = nadirGuidance(lvlhFrame.dcmIL)
    else:
        # OFF guidance mode or current mode is not defined
        fswGuidanceParam.MODE = "GUIDMODE_OFF"
        (angRateGuid_B, eulerAngGuid_BI) = offGuidance()   
    
    # Update output bus signals
    fswGuidBusOut.signals["angRateGuid_BI_B"].update(angRateGuid_B)
    fswGuidBusOut.signals["eulerAngGuid_BI"].update(eulerAngGuid_BI)

    return fswGuidBusOut 


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------

