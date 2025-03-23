# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np
import ephem
import src.utils.conversions as conversions
import src.attitudeKinematics as attKin
import src.attitudeDynamics as attDyn
from src.attitudeKinematics import Rx as Rx
from src.attitudeKinematics import Ry as Ry
from src.attitudeKinematics import Rz as Rz

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def computeEphemerides(simBus, modelsBus, simParam):
    # Initialize outputs bus
    modelsEnvBusOut = modelsBus.subBuses["environment"]

    # Retrieve inputs / parameters
    elapsedTime = simBus.signals["elapsedTime"].value
    dateTimeStart = simParam.dateTimeStart
    ephemEpoch = simParam.ephemEpoch
    # Compute ephemerides
    currDateTime = ephem.date(dateTimeStart + elapsedTime/(24*3600))
    marsEphem = ephem.Mars()
    marsEphem.compute(currDateTime, epoch = ephemEpoch)
    # Compute directions
    sunToMarsDir_I = conversions.latLonToCartesian(marsEphem.hlat/deg2rad, marsEphem.hlon/deg2rad, 1)

    # Update output bus signals
    modelsEnvBusOut.signals["sunToMarsDir_I"].update(sunToMarsDir_I)
    return modelsEnvBusOut
    

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


def computeExtTorque(simParam, attDynParam, orbit, modelsBus):
    # Initialize output bus
    modelsBusOut = modelsBus

    # Retrieve signals / parameters of interest
    qLI_sca = modelsBus.subBuses["environment"].signals["qLI_sca"].value
    qLI_vec = modelsBus.subBuses["environment"].signals["qLI_vec"].value
    qBI_sca = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].value
    qBI_vec = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].value
    angRate_BI_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].value

    # Reconstruct the dynamic state and LVLH frame quaternion
    qLI = attKin.Quaternion(qLI_sca, qLI_vec)
    qBI = attKin.Quaternion(qBI_sca, qBI_vec)
    myDynState = attDyn.DynamicsState(qBI, angRate_BI_B)

    # Gravity gradient disturbance torque
    torqueGG_B = computeGGTorque(simParam, attDynParam, orbit.orbitContext.orbitPulse, myDynState.qBI, qLI)
    # Aerodynamic disturbance torque
    torqueAero_B = computeAeroTorque()
    # Total disturbance torque
    torqueExt_B = torqueGG_B + torqueAero_B

    # Update output bus signals
    modelsBusOut.subBuses["environment"].signals["torqueGG_B"].update(torqueGG_B)
    modelsBusOut.subBuses["environment"].signals["torqueAero_B"].update(torqueAero_B)
    modelsBusOut.subBuses["environment"].signals["torqueExt_B"].update(torqueExt_B)

    return modelsBusOut


# --------------------------------------------------
# SIMULATION / TEST
# -------------------------------------------------- 