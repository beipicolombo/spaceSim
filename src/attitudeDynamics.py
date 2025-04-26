# -*- coding: utf-8 -*-
"""
Created on Mon Mar 28 23:27:46 2022

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

# Attitude dynamics state
class DynamicsState():
    def __init__(self, qBI = attitudeKinematics.Quaternion(), angRate_BI_B = np.array([0, 0, 0])):
        # Initial attitude and angular rates
        self.qInitVec = attitudeKinematics.Quaternion()
        self.angRateInit_B = np.array([0, 0, 0])
        # Current attitude and angular rates
        self.angRate_BI_B = angRate_BI_B
        self.qBI = qBI
            
    def propagateState(self, scMassParam, simParam, torqueTot_B):
        self.qBI.propagateState(self.angRate_BI_B, simParam)
        self.angRate_BI_B = propagateAngRates(self.angRate_BI_B, torqueTot_B, scMassParam, simParam)
    
# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def propagateAttitude(scMassParam, simParam, modelsBus):
    # Initialize output bus
    modelsBusOut = modelsBus
    # Retrieve inputs
    torqueTot_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["torqueTot_B"].value
    qBI_sca = modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].value
    qBI_vec = modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].value
    angRate_BI_B = modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].value
    # Reconstruct quaternion and dynamics state
    qBI = attitudeKinematics.Quaternion(qBI_sca, qBI_vec)
    dynState = DynamicsState(qBI, angRate_BI_B)
    # Propagate
    dynState.propagateState(scMassParam, simParam, torqueTot_B)
    # Set output bus signals
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].update(dynState.qBI.sca)
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].update(dynState.qBI.vec)
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].update(dynState.angRate_BI_B)
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].update(dynState.qBI.toEuler())

    return modelsBusOut


def setInitialAttitude(modelsBus, simParam, dynState):
    # Initialize output bus
    modelsBusOut = modelsBus

    # Retrieve inputs
    qInitVec = dynState.qInitVec
    angRateInit_B = dynState.angRateInit_B
    swInitAttitudeFrame = simParam.simOptions.swInitAttitudeFrame
    qInit = attitudeKinematics.trans_VecToQuat(qInitVec, isScalarFirst = True)

    if (swInitAttitudeFrame == "L"):
        # Initial attitude is initialized wrt nadir reference frame
        qLI_sca = modelsBus.subBuses["environment"].signals["qLI_sca"].value 
        qLI_vec = modelsBus.subBuses["environment"].signals["qLI_vec"].value
        qLI = attitudeKinematics.Quaternion(qLI_sca, qLI_vec)
        angRate_LI_L = modelsBus.subBuses["environment"].signals["angRate_LI_L"].value
        angRate_BL_B = angRateInit_B
        qBL = qInit
        # Initialize attitude and angular rates
        angRate_BI_B = angRate_BL_B + np.matmul(qBL.toDcm(), angRate_LI_L)
        qBI = attitudeKinematics.multiplyQuat(qLI, qBL)
    elif (swInitAttitudeFrame == "J"):
        # Initial attitude is initialized wrt inertial reference frame
        angRate_BI_B = angRateInit_B
        qBI = qInit
    else:
        # Initial attitude is initialized wrt inertial reference frame by default
        angRate_BI_B = np.zeros(3)
        qBI = attitudeKinematics.Quaternion()
    qBI = qBI.normalize()    

    # Update output bus signals
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].update(qBI.sca)
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].update(qBI.vec)
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].update(angRate_BI_B)
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].update(qBI.toEuler())

    return modelsBusOut
    
# Angular acceleration
def angRatesDot(angRates, externalTorque, attDynParam):   
    scInertia    = attDynParam.inertiaSc_B
    scInertiaInv = attDynParam.inertiaScInv_B
    
    angRatesDot  = np.matmul(scInertiaInv, externalTorque - np.cross(angRates, np.matmul(scInertia, angRates)))
    
    return angRatesDot

# Attitude quaternion propagation

# Angular rates propagation
def propagateAngRates(angRatesPrev, externalTorquePrev, attDynParam, simParam):
    xPrev = angRatesPrev
    Ts = simParam.Ts
    
    k1 = angRatesDot(xPrev, externalTorquePrev, attDynParam)
    x2 = xPrev + Ts/2 * k1
    k2 = angRatesDot(x2, externalTorquePrev, attDynParam)
    x3 = xPrev + Ts/2 * k2
    k3 = angRatesDot(x3, externalTorquePrev, attDynParam)
    x4 = xPrev + Ts * k3
    k4 = angRatesDot(x4, externalTorquePrev, attDynParam)
        
    angRatesNext = xPrev + Ts/6 * (k1 + 2*k2 + 2*k3 + k4)
     
    return angRatesNext
    
# Total total external torque
def computeTotTorque(modelsBus):
    # Retrieve useful inputs
    torqueExt_B = modelsBus.subBuses["environment"].signals["torqueExt_B"].value
    torqueAct_B = modelsBus.subBuses["actuators"].signals["torqueAct_B"].value

    # Initialize output bus
    modelsBusOut = modelsBus

    # Compute total external torque
    torqueTot_B = torqueExt_B + torqueAct_B
    
    # Update output bus signals
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["torqueTot_B"].update(torqueTot_B)

    return modelsBusOut

# Angular momentum
def computeAngMom(attDynParam, modelsBus):
    # Retrieve useful inputs
    # angMomRw_B = np.array([0, 0, 0]) # RW models to be implemented
    angRate_BI_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].value
    inertiaSc_B = attDynParam.inertiaSc_B
    
    # Initialize output bus
    modelsBusOut = modelsBus
    
    # Compute S/C angular momentum
    angMomSc_B = np.matmul(inertiaSc_B, angRate_BI_B)
    
    # Update output bus signals
    modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].update(angMomSc_B)
    
    return modelsBusOut
    

# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------