# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np
import math as m
import src.attitudeKinematics as attitudeKinematics

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class LVLHframe:
    def __init__(self):
        self.dcmLI = np.identity(3)
        self.zL_B = np.array([1, 0, 0])
        self.zL_I = np.array([1, 0, 0])
        self.eulerAng_LI = np.array([0, 0, 0])

    def update(self, orbit, dynState):
        zL_L = np.array([0, 0, 1])

        # Get the DCM from inertial frame
        posOnOrbit = orbit.keplerElem.ta
        dcmLI = np.matmul(attitudeKinematics.Rx(-np.pi/2), attitudeKinematics.Rz(pi/2 + posOnOrbit))
        eulerAng_LI = np.array([-np.pi/2, 0, pi/2 + posOnOrbit])
        qLI = attitudeKinematics.trans_EulerAngToQuat(eulerAng_LI)

        zL_I = np.matmul(np.transpose(dcmLI), zL_L)
        zL_B = attitudeKinematics.applyRotation(dynState.qBI, zL_I)

        self.dcmLI = dcmLI
        self.zL_B = zL_B
        self.zL_I = zL_I
        self.eulerAng_LI
        self.qLI = qLI

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

def computeGGTorque(simParam, attDynParam, orbitPulse, zL_B):
    # Compute gravity gradient torque
    if simParam.simOptions.isGGTorqueEnabled:
        torqueGG_B = 3*pow(orbitPulse,2) * np.matmul(attitudeKinematics.trans_VecToCrossMat(zL_B), np.matmul(attDynParam.inertiaSc_B, zL_B))
    else:
        torqueGG_B = np.array([0, 0, 0])

    return torqueGG_B


def computeAeroTorque():
    # TBW
    return np.array([0, 0, 0])


def computeExtTorque(simParam, attDynParam, orbit, lvlhFrame, envBusIn):
    # Initialize output
    envBusOut = envBusIn

    # Gravity gradient disturbance torque
    torqueGG_B = computeGGTorque(simParam, attDynParam, orbit.orbitContext.orbitPulse, lvlhFrame.zL_B)
    # Aerodynamic disturbance torque
    torqueAero_B = computeAeroTorque()
    # Total disturbance torque
    torqueExt_B = torqueGG_B + torqueAero_B

    # Update output bus signals
    envBusOut.signals["torqueGG_B"].update(torqueGG_B)
    envBusOut.signals["torqueAero_B"].update(torqueAero_B)
    envBusOut.signals["torqueExt_B"].update(torqueExt_B)

    return envBusOut


# --------------------------------------------------
# SIMULATION / TEST
# -------------------------------------------------- 