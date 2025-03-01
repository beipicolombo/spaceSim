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
        self.attControlKp = 0.1
        self.attControlKd = 10

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def thrustControl(fswControlParam):
    torqueCtrl_B = np.array([0, 0, 0]) 
    forceCtrl_B = np.array([0, 0, 0])
    return forceCtrl_B

def rateControl(fswControlParam, angRateEst_B, angRateGuid_B):
    torqueCtrl_B = fswControlParam.rateDampingKd * (angRateGuid_B - angRateEst_B)
    
    return torqueCtrl_B

def attitudeControl(fswControlParam, angRateMeas_B, eulerAngMeas_BI, angRateGuid_B, eulerAngGuid_BI):
    torqueCtrl_B = fswControlParam.attControlKd * (angRateGuid_B - angRateMeas_B) + fswControlParam.attControlKp * (eulerAngGuid_BI - eulerAngMeas_BI)
    
    return torqueCtrl_B

def offControl() :
    forceCtrl_B = np.array([0, 0, 0])
    torqueCtrl_B = np.array([0, 0, 0])
    return (forceCtrl_B, torqueCtrl_B)

def computeControl(fswControlParam, fswBus):
    # Retrieve useful inputs
    angRateEst_B =  fswBus.subBuses["estimation"].signals["angRateEst_BI_B"].value
    eulerAngEst_BI =  fswBus.subBuses["estimation"].signals["eulerAngEst_BI"].value
    angRateGuid_B = fswBus.subBuses["guidance"].signals["angRateGuid_BI_B"].value
    eulerAngGuid_BI = fswBus.subBuses["guidance"].signals["eulerAngGuid_BI"].value
    aocsCtrMode = fswBus.subBuses["modeMgt"].signals["aocsCtrMode"].value
    aocsCtrActMode = fswBus.subBuses["modeMgt"].signals["aocsCtrActMode"].value

    # Initialize output bus
    fswControlBusOut = fswBus.subBuses["control"]

    # Compute control toraque depending on the current control mode
    if (aocsCtrMode == "CTRLMODE_OFF"):
        # OFF control mode
        (forceCtrl_B, torqueCtrl_B) = offControl()
    elif (aocsCtrMode == "CTRLMODE_RATE_DAMP_CTRL"):
        # Rate damping control mode
        torqueCtrl_B = rateControl(fswControlParam, angRateEst_B, angRateGuid_B)
        forceCtrl_B = np.array([0, 0, 0])
    elif (aocsCtrMode == "CTRLMODE_ATT_CTRL"):
        # Inertial attitude control mode
        torqueCtrl_B = attitudeControl(fswControlParam, angRateEst_B, eulerAngEst_BI, angRateGuid_B, eulerAngGuid_BI)
        forceCtrl_B = np.array([0, 0, 0])
    elif (aocsCtrMode == "CTRLMOD_THRUST_CTRL"):
        torqueCtrl_B = attitudeControl(fswControlParam, angRateEst_B, eulerAngEst_BI, angRateGuid_B, eulerAngGuid_BI)
        forceCtrl_B = np.array([0, 0, 0])
    else:
        # OFF control mode or current mode is not defined
        (forceCtrl_B, torqueCtrl_B) = offControl()

    
    # Switch between THR and RW control torques
    if (aocsCtrActMode == "THR"):
        # THR only
        torqueCtrlThr_B = torqueCtrl_B
        forceCtrlThr_B = forceCtrl_B
        torqueCtrlRw_B = np.array([0, 0, 0])
    elif (aocsCtrActMode == "RW"):
        # RW only
        torqueCtrlThr_B = np.array([0, 0, 0])
        forceCtrlThr_B = np.array([0, 0, 0])
        torqueCtrlRw_B = torqueCtrl_B
    else :
        # RW_OFFLOADING => TBW
        torqueCtrlThr_B = np.array([0, 0, 0])
        forceCtrlThr_B = np.array([0, 0, 0])
        torqueCtrlRw_B = np.array([0, 0, 0])
        
    # Update output bus signals
    fswControlBusOut.signals["forceCtrl_B"].update(forceCtrl_B)
    fswControlBusOut.signals["torqueCtrl_B"].update(torqueCtrl_B)
    fswControlBusOut.signals["forceCtrlThr_B"].update(forceCtrlThr_B)
    fswControlBusOut.signals["torqueCtrlThr_B"].update(torqueCtrlThr_B)
    fswControlBusOut.signals["torqueCtrlRw_B"].update(torqueCtrlRw_B)
        
    return (fswControlBusOut)

# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------