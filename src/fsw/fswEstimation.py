# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np
import src.attitudeKinematics as attitudeKinematics

deg2rad = np.pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------

class FswEstimationParam:
    # To be moved as higher level class
    def __init__(self):
        self.Dummy = 0


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def computeEstimation(fswEstimationParam, fswBusIn, modelsBus):
    # Initialize output bus
    fswEstBusOut = fswBusIn.subBuses["estimation"]

    # Position and velocity estimation
    (posEst_J, velEst_J, qLIest, angRateEst_LI_L) = positionVelocityEstimation(fswEstimationParam, fswBusIn, modelsBus)
    # Attitude estimation
    (angRateEst_BI_B, eulerAngEst_BI, qBIest) = attitudeEstimation(fswEstimationParam, fswBusIn, modelsBus)

    # Update output bus signals
    fswEstBusOut.signals["posEst_J"].update(posEst_J)
    fswEstBusOut.signals["velEst_J"].update(velEst_J)
    fswEstBusOut.signals["qLIest_sca"].update(qLIest.sca)
    fswEstBusOut.signals["qLIest_vec"].update(qLIest.vec)
    fswEstBusOut.signals["angRateEst_LI_L"].update(angRateEst_LI_L)

    fswEstBusOut.signals["angRateEst_BI_B"].update(angRateEst_BI_B)
    fswEstBusOut.signals["eulerAngEst_BI"].update(eulerAngEst_BI)
    fswEstBusOut.signals["qBIest_sca"].update(qBIest.sca)
    fswEstBusOut.signals["qBIest_vec"].update(qBIest.vec)

    return fswEstBusOut


def positionVelocityEstimation(fswEstimationParam, fswBus, modelsBus):
    # Retrieve useful inputs
    pos_J = modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].value
    vel_J = modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["vel_I"].value
    qLI_sca = modelsBus.subBuses["environment"].signals["qLI_sca"].value
    qLI_vec = modelsBus.subBuses["environment"].signals["qLI_vec"].value
    angRate_LI_L = modelsBus.subBuses["environment"].signals["angRate_LI_L"].value

    # Initialize output bus
    fswEstBusOut = fswBus.subBuses["estimation"]

    # Position estimation
    posEst_J = pos_J
    # Velocity estimation
    velEst_J = vel_J
    # Nadir frame estimation
    qLIest_sca = qLI_sca 
    qLIest_vec = qLI_vec
    qLIest = attitudeKinematics.Quaternion(qLIest_sca, qLIest_vec)
    angRateEst_LI_L = angRate_LI_L

    return (posEst_J, velEst_J, qLIest, angRateEst_LI_L)


def attitudeEstimation(fswEstimationParam, fswBus, modelsBus):
    # Retrieve useful inputs
    angRateMeas_BI_B = modelsBus.subBuses["sensors"].signals["angRateMeas_BI_B"].value
    eulerAngMeas_BI = modelsBus.subBuses["sensors"].signals["eulerAngMeas_BI"].value
    qBImeas_sca = modelsBus.subBuses["sensors"].signals["qBImeas_sca"].value
    qBImeas_vec = modelsBus.subBuses["sensors"].signals["qBImeas_vec"].value

    # Initialize output bus
    fswEstBusOut = fswBus.subBuses["estimation"]

    # Angular rates estimation
    angRateEst_BI_B = angRateMeas_BI_B
    # Attitude estimation
    eulerAngEst_BI = eulerAngMeas_BI
    qBIest = attitudeKinematics.Quaternion(qBImeas_sca, qBImeas_vec).normalize()
    
    return (angRateEst_BI_B, eulerAngEst_BI, qBIest)


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------

