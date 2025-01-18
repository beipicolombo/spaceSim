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

# Attitude dynamics parameters
class AttDynParam:
    def __init__(self):
        # inertial at the geometric frame origin, in geometric frame => TBW
        self.inertialSc_G = np.identity(3)
        # inertia at the center of mass, in body frame [kg*m^2]
        self.inertiaSc_B  = np.identity(3)
        self.inertiaScInv_B = np.linalg.inv(self.inertiaSc_B)
        # S/C mass [kg]
        self.massSc  = 10
        # S/C CB mass [kg] (equal to S/C total mass by default) => appendages to be added
        self.massCB = self.massSc
        # S/C center of mass position in geometric frame [m]
        self.posComSc_G = np.zeros(3)

    def printAttributes(self):
        print("=======================================")
        print("Attitude dynamics parameters") 
        print("inertia at the center of mass, in body frame [kg*m^2]")
        print(self.inertiaSc_B)
        print("S/C mass [kg]")
        print(self.massSc)
        print("S/C CB mass [kg]")
        print(self.massCB)
        print("S/C center of mass position in geometric frame [m]")
        print(self.comPosSc_B0)
            

# Attitude dynamics state
class DynamicsState():
    def __init__(self):
        # Angular rates between S/C and inertial frame in S/C frame
        angRate_BI_B = np.zeros(3)
        # Angular rates between S/c and LVLV frame in S/C frame
        angRate_BL_B = np.zeros(3)
        # Attitude quaternion between S/C frame and inetial frame
        qBI = attitudeKinematics.Quaternion(1, np.array([0, 0, 0]))
        qBI = qBI.normalize()

        self.angRate_BI_B = angRate_BI_B
        self.angRate_BL_B = angRate_BL_B
        self.qBI = qBI 
            
    def propagateState(self, attDynParam, simParam, modelsBus):
        # Retrieve useful inputs
        torqueTot_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["torqueTot_B"].value
        # Propagate state
        self.qBI.propagateState(self.angRate_BI_B, simParam)
        self.angRate_BI_B = propagateAngRates(self.angRate_BI_B, torqueTot_B, attDynParam, simParam)
    
# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

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

# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------