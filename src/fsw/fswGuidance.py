# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np
import src.attitudeKinematics as attitudeKinematics

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# CLASSES
# --------------------------------------------------

class FswGuidanceParam:
    # To be moved as higher level class
    def __init__(self):
        self.GUIDMODE_ATT_INERT_eulerAngGuid_RI = np.array([0, 0, 0])*deg2rad
        self.orbitRate = 0 # [rad/s]


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

def rateDampingGuidance():
    angRate_RI_R = np.array([0, 0, 0])
    qRI = attitudeKinematics.Quaternion()
    return (angRate_RI_R, qRI)

def attitudeInertialGuidance(fswGuidanceParam):
    angRate_RI_R = np.array([0, 0, 0])
    qRI = attitudeKinematics.trans_EulerAngToQuat(fswGuidanceParam.GUIDMODE_ATT_INERT_eulerAngGuid_RI)
    return (angRate_RI_R, qRI)

def offGuidance():
    angRate_RI_R = np.array([0, 0, 0])
    qRI = attitudeKinematics.Quaternion()
    return (angRate_RI_R, qRI)

def nadirGuidance(fswGuidanceParam, angRate_LI_L, qLI):
    angRate_RI_R = angRate_LI_L
    qRI = qLI
    return (angRate_RI_R, qRI)

def computeGuidance(fswGuidanceParam, envBusIn, fswBus):
    # Initialize output bus
    fswGuidBusOut = fswBus.subBuses["guidance"]

    # Retrieve signals / parameters
    aocsGuidMode = fswBus.subBuses["modeMgt"].signals["aocsGuidMode"].value
    angRate_LI_L = envBusIn.signals["angRate_LI_L"].value
    qLI_sca = envBusIn.signals["qLI_sca"].value
    qLI_vec = envBusIn.signals["qLI_vec"].value

    # Reconstruct the LVLH frame quaternion
    qLI = attitudeKinematics.Quaternion(qLI_sca, qLI_vec)

    # Compute guidance angular rates and euler angles depending on the current guidance mode
    if (aocsGuidMode == "GUIDMODE_ATT_INERT"):
        # Inertial attitude guidance mode
        (angRate_RI_R, qRI) = attitudeInertialGuidance(fswGuidanceParam)
    elif (aocsGuidMode == "GUIDMODE_RATE_DAMPING"):
        # Rate damping guidance mode
        (angRate_RI_R, qRI) = rateDampingGuidance()
    elif (aocsGuidMode == "GUIDMODE_ATT_NADIR"):
        # Nadir guidance mode
        (angRate_RI_R, qRI) = nadirGuidance(fswGuidanceParam, angRate_LI_L, qLI)
    else:
        # OFF guidance mode or current mode is not defined
        aocsGuidMode = "GUIDMODE_OFF"
        (angRate_RI_R, qRI) = offGuidance()   
    
    qRI.normalize()
    
    # Compute guidance with respect to the nadir reference frame
    qRL = attitudeKinematics.multiplyQuat(qLI.conjugate(), qRI)

    # Compute euler angles
    eulerAng_RI = qRI.toEuler() 
    eulerAng_RL = qRL.toEuler()

    fswGuidBusOut.signals["eulerAng_RI"].update(eulerAng_RI)
    fswGuidBusOut.signals["eulerAng_RL"].update(eulerAng_RL)
    fswGuidBusOut.signals["qRI_sca"].update(qRI.sca)
    fswGuidBusOut.signals["qRI_vec"].update(qRI.vec)
    fswGuidBusOut.signals["angRate_RI_R"].update(angRate_RI_R)

    return fswGuidBusOut 


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------

