# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# Thrusters model parameters
class ThrModelParam:
    def __init__(self, attDynParam):
        self.nbThrSet = 0
        self.thrSets = []
        for ii in range(1, self.nbThrSet+1):
            self.thrSets.append(ThrSetParam(attDynParam))

    def updateThrSets(self, nbThrSet, attDynParam):
        self.nbThrSet = nbThrSet
        for ii in range(1, self.nbThrSet+1):
            self.thrSets.append(ThrSetParam(attDynParam))

    def computeThrTorque(self, fswCtrlBus):
        torqueCtrl_B = np.array(fswCtrlBus.signals["torqueCtrl_B"].value)
        torqueThr_B = np.array([0, 0, 0])
        if (self.nbThrSet > 0):
            for thrSetParam in self.thrSets:
                torqueThr_B = torqueThr_B + thrSetParam.computeThrSetTorque(torqueCtrl_B)/self.nbThrSet
        
        return torqueThr_B

# Thrusters set parameters
class ThrSetParam:
    def __init__(self, attDynParam):
        self.nbThr = 0
        self.thrUnits = []
        for ii in range(1, self.nbThr+1):
            self.thrUnits.append(ThrUnitParam(ii))
        self.inflMat_B = self.getInfluenceMatrix(attDynParam)
        self.isOn = False

    def updateThrSet(self, nbThr, attDynParam):
        self.nbThr = nbThr
        for ii in range(1, self.nbThr+1):
            self.thrUnits.append(ThrUnitParam(ii))
        self.inflMat_B = self.getInfluenceMatrix(attDynParam)

    def computeThrSetTorque(self, torqueCtrl_B):
        if self.isOn:
            torqueThrSet_B = torqueCtrl_B
        else:
            torqueThrSet_B = np.array([0, 0, 0])

        return torqueThrSet_B

    # Compute the thrusters set influence matrix
    def getInfluenceMatrix(self, attDynParam):
        matOfInfluence_B = np.zeros([6, self.nbThr])
        posComSc_G = attDynParam.posComSc_G

        for ii in range(self.nbThr):
            dirUnit_G = self.thrUnits[ii].dirUnit_G
            posUnit_B = self.thrUnits[ii].posUnit_G - posComSc_G
            forceUnit = self.thrUnits[ii].forceUnit
            matOfInfluence_B[0:3, ii] = dirUnit_G*forceUnit
            matOfInfluence_B[3:6, ii] = np.cross(posUnit_B, dirUnit_G*forceUnit)

        return matOfInfluence_B

# Thruster unit parameters
class ThrUnitParam:
    def __init__(self, thrId):

        self.thrId = thrId
        self.posUnit_G = np.array([0, 0, 0]) # [-]
        self.dirUnit_G = np.array([0, 0, -1]) # [-]
        self.forceUnit = 10 # [N]


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------



# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------