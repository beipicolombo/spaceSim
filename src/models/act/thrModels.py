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
class Thr:
    def __init__(self):
        self.nbSets = 0
        self.sets = {}

    def addThrSet(self, setName, nbUnits):
        setId = self.nbSets + 1
        self.nbSets += 1
        self.sets.update({setName: ThrSet(name = setName, id = setId, nbUnits = nbUnits)})

    def computeTrq(self, fswCmdBus):
        torqueCmd_B = np.array(fswCmdBus.signals["torqueCmdThr_B"].value)
        torqueThr_B = np.array([0, 0, 0])
        if (self.nbSets > 0):
            for thrSet in self.sets:
                torqueThr_B = torqueThr_B + thrSet.computeTrq(torqueCmd_B)/self.nbSets 
        return torqueThr_B


class ThrSet():
    def __init__(self, name = "empty", id = 0, nbUnits = 0):
        self.name = name
        self.id = id
        self.nbUnits = nbUnits
        self.units = {}
        for kk in range(1, nbUnits+1):
            unitName = ("unit"+str(kk))
            self.units.update({unitName: ThrUnit(name = unitName, id = kk)})

    # Compute the thrusters set influence matrix
    # def getInfluenceMatrix(self, attDynParam):
    #     matOfInfluence_B = np.zeros([6, self.nbUnits])
    #     posComSc_G = attDynParam.posComSc_G

    #     for ii in range(self.nbUnits):
    #         dirUnit_G = self.units[ii].dirUnit_G
    #         posUnit_B = self.units[ii].posUnit_G - posComSc_G
    #         forceUnit = self.units[ii].forceUnit
    #         matOfInfluence_B[0:3, ii] = dirUnit_G*forceUnit
    #         matOfInfluence_B[3:6, ii] = np.cross(posUnit_B, dirUnit_G*forceUnit)

    #     return matOfInfluence_B

    def computeTrq(self, torqueCmd_B):
        torque_B = torqueCmd_B
        return torque_B


class ThrUnit():
    def __init__(self, name = "empty", id = 0):
        self.name = name
        self.id = id
        self.posUnit_G = np.array([0, 0, 0]) # [-]
        self.dirUnit_G = np.array([0, 0, -1]) # [-]
        self.forceUnit = 10 # [N]


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
# [TBW] Consumption model



# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------