# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np
import math as m
import src.attitudeKinematics as attKin
from src.attitudeKinematics import Rx as Rx
from src.attitudeKinematics import Ry as Ry
from src.attitudeKinematics import Rz as Rz

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class LVLHframe:
    def __init__(self):
        self.dcmLI = np.identity(3)
        self.eulerAng_LI = np.array([0, 0, 0])
        self.qLI = attKin.Quaternion()
        self.angRate_LI_L = np.array([0, 0, 0])

    def update(self, orbit):
        # Get the DCM from inertial frame
        raan = orbit.keplerElem.raan
        inc = orbit.keplerElem.inc
        argPer = orbit.keplerElem.argPer
        ta = orbit.keplerElem.ta

        #xVec = np.array([1, 0, 0])
        #yVec = np.array([0, 1, 0])
        #zVec = np.array([0, 0, 1])
        #quat = attKin.trans_AngVecToQuat(raan, zVec)
        #quat = attKin.multiplyQuat(attKin.trans_AngVecToQuat(inc, xVec), quat)
        #quat = attKin.multiplyQuat(attKin.trans_AngVecToQuat(argPer+ta, zVec), quat)
        #quat = attKin.multiplyQuat(attKin.trans_AngVecToQuat(-np.pi/2, yVec), quat)
        #qLI  = attKin.multiplyQuat(attKin.trans_AngVecToQuat(np.pi/2, zVec), quat)
        #dcmLI = qLI.toDcm()
        #eulerAng_LI = qLI.toEuler()

        mat = np.matmul(Rx(inc), Rz(raan))
        mat = np.matmul(Rz(argPer+ta), mat)
        mat = np.matmul(Ry(-np.pi/2), mat)
        dcmLI = np.matmul(Rz(np.pi/2), mat)
        qLI = attKin.trans_DcmToQuat(dcmLI).normalize()
        eulerAng_LI = qLI.toEuler()

        self.dcmLI = dcmLI
        self.eulerAng_LI = eulerAng_LI
        self.qLI = qLI
        self.angRate_LI_L = 0.001*np.array([0, -orbit.orbitContext.orbitPulse, 0])

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def computeLvlhFrame(orbit, modelsBus):
    # Initialize output
    modelsBusOut = modelsBus

    # Retrieve signals / parameters of interest
    inc  = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["inc"].value
    raan = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["raan"].value
    argPer = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["argPer"].value
    ta = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["ta"].value

    # Compute attitude
    mat = np.matmul(Rx(inc), Rz(raan))
    mat = np.matmul(Rz(argPer+ta), mat)
    mat = np.matmul(Ry(-np.pi/2), mat)
    dcmLI = np.matmul(Rz(np.pi/2), mat)
    qLI = attKin.trans_DcmToQuat(dcmLI).normalize()
    eulerAng_LI = qLI.toEuler()
    # Compute angular rates
    angRate_LI_L = np.array([0, -orbit.orbitContext.orbitPulse, 0])

    # Update output bus signals
    modelsBusOut.subBuses["environment"].signals["eulerAng_LI"].update(eulerAng_LI)
    modelsBusOut.subBuses["environment"].signals["qLI_sca"].update(qLI.sca)
    modelsBusOut.subBuses["environment"].signals["qLI_vec"].update(qLI.vec)
    modelsBusOut.subBuses["environment"].signals["angRate_LI_L"].update(angRate_LI_L)

    return modelsBusOut


def computeGGTorque(simParam, attDynParam, orbitPulse, qBI, qLI):
    dcmLI = qLI.toDcm()

    # Compute the Z axis of the LVLH frame in body frame
    zL_L = np.array([0, 0, 1])
    zL_I = np.matmul(np.transpose(dcmLI), zL_L)
    zL_B = attKin.applyRotation(qBI, zL_I)

    # Compute gravity gradient torque
    if simParam.simOptions.isGGTorqueEnabled:
        torqueGG_B = 3*pow(orbitPulse,2) * np.matmul(attKin.trans_VecToCrossMat(zL_B), np.matmul(attDynParam.inertiaSc_B, zL_B))
    else:
        torqueGG_B = np.array([0, 0, 0])

    return torqueGG_B


def computeAeroTorque():
    # TBW
    return np.array([0, 0, 0])


def computeExtTorque(simParam, attDynParam, myDynState, orbit, envBusIn):
    # Initialize output bus
    envBusOut = envBusIn

    # Retrieve signals / parameters of interest
    qLI_sca = envBusIn.signals["qLI_sca"].value
    qLI_vec = envBusIn.signals["qLI_vec"].value

    # Reconstruct the LVLH frame quaternion
    qLI = attKin.Quaternion(qLI_sca, qLI_vec)

    # Gravity gradient disturbance torque
    torqueGG_B = computeGGTorque(simParam, attDynParam, orbit.orbitContext.orbitPulse, myDynState.qBI, qLI)
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