# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np


# --------------------------------------------------
# CLASSES
# --------------------------------------------------

class FswControlParam:
    # To be moved as higher level class
    def __init__(self):
        self.rateDampingKd = 10.0
        self.attControlKp = 0.01
        self.attControlKd = 0.1
        self.MODE = FswControlMode()
   
class FswControlMode:
    def __init__(self):
        self.MODE = "CTRLMODE_OFF"


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

def rateControl(fswControlParam, angRateEst_B, angRateGuid_B):
    torqueCtrl_B = fswControlParam.rateDampingKd * (angRateGuid_B - angRateEst_B)
    
    return torqueCtrl_B

def attitudeControl(fswControlParam, angRateMeas_B, eulerAngMeas_BI, angRateGuid_B, eulerAngGuid_BI):
    torqueCtrl_B = fswControlParam.attControlKd * (angRateGuid_B - angRateMeas_B) + fswControlParam.attControlKp * (eulerAngGuid_BI - eulerAngMeas_BI)
    
    return torqueCtrl_B

def offControl() :
    return np.array([0, 0, 0])

def computeControl(fswControlParam, fswBus):
    # Retrieve useful inputs
    angRateEst_B =  fswBus.subBuses["estimation"].signals["angRateEst_BI_B"].value
    eulerAngEst_BI =  fswBus.subBuses["estimation"].signals["eulerAngEst_BI"].value
    angRateGuid_B = fswBus.subBuses["guidance"].signals["angRateGuid_BI_B"].value
    eulerAngGuid_BI = fswBus.subBuses["guidance"].signals["eulerAngGuid_BI"].value

    # Initialize output bus
    fswControlBusOut = fswBus.subBuses["control"]

    # Compute control toraque depending on the current control mode
    if (fswControlParam.MODE == "CTRLMODE_OFF"):
        # OFF control mode
        torqueCtrl_B = offControl()
    elif (fswControlParam.MODE == "CTRLMODE_RATE_DAMP_CTRL"):
        # Rate damping control mode
        torqueCtrl_B = rateControl(fswControlParam, angRateEst_B, angRateGuid_B)
    elif (fswControlParam.MODE == "CTRLMODE_ATT_CTRL"):
        # Inertial attitude control mode
        torqueCtrl_B = attitudeControl(fswControlParam, angRateEst_B, eulerAngEst_BI, angRateGuid_B, eulerAngGuid_BI)
    else:
        # OFF control mode or current mode is not defined
        torqueCtrl_B = offControl()
        
    # Update output bus signals
    fswControlBusOut.signals["torqueCtrl_B"].update(torqueCtrl_B)
    
    return fswControlBusOut


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------