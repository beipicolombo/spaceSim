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
        self.GUIDMODE_ATT_INERT_eulerAngGuid_BI = np.array([0, 0, 0])*deg2rad
        self.orbitRate = 0 # [rad/s]


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

def nadirGuidance(fswGuidanceParam, eulerAng_LI):
    angRateGuid_B = np.array([0, -fswGuidanceParam.orbitRate, 0])
    eulerAngGuid_BI = eulerAng_LI
    
    return (angRateGuid_B, eulerAngGuid_BI)

def computeGuidance(fswGuidanceParam, lvlhFrame, modelsBus, fswBus):
    # Retrieve useful inputs
    zL_B = modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_B"].value
    zL_I = modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_I"].value
    aocsGuidMode = fswBus.subBuses["modeMgt"].signals["aocsGuidMode"].value
    
    # Initialize output bus
    fswGuidBusOut = fswBus.subBuses["guidance"]

    # Compute guidance angular rates and euler angles depending on the current guidance mode
    if (aocsGuidMode == "GUIDMODE_ATT_INERT"):
        # Inertial attitude guidance mode
        (angRateGuid_B, eulerAngGuid_BI) = attitudeInertialGuidance(fswGuidanceParam)
    elif (aocsGuidMode == "GUIDMODE_RATE_DAMPING"):
        # Rate damping guidance mode
        (angRateGuid_B, eulerAngGuid_BI) = rateDampingGuidance()
    elif (aocsGuidMode == "GUIDMODE_ATT_NADIR"):
        # Nadir guidance mode
        (angRateGuid_B, eulerAngGuid_BI) = nadirGuidance(fswGuidanceParam, lvlhFrame.eulerAng_LI)
    else:
        # OFF guidance mode or current mode is not defined
        aocsGuidMode = "GUIDMODE_OFF"
        (angRateGuid_B, eulerAngGuid_BI) = offGuidance()   
    
    # Update output bus signals
    fswGuidBusOut.signals["angRateGuid_BI_B"].update(angRateGuid_B)
    fswGuidBusOut.signals["eulerAngGuid_BI"].update(eulerAngGuid_BI)

    return fswGuidBusOut 


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------

