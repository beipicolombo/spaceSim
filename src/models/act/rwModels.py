# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np
# import math as m
# import src.attitudeKinematics as attitudeKinematics

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# Reaction wheels model parameters
class Param:
    def __init__(self):
        self.nbRwSet = 0
        self.rwSets = []
        for ii in range(1, self.nbRwSet+1):
            self.rwSets.append(RwSetParam())

    def updateRwSets(self, nbRwSet):
        self.nbRwSet = nbRwSet
        for ii in range(1, self.nbRwSet+1):
            self.rwSets.append(RwSetParam())

    def computeRwTorque(self):
        torqueRw_B = np.array([0, 0, 0])
        return torqueRw_B

# RW set parameters:
class RwSetParam:
    def __init__(self):
        self.nbRw = 0
        self.rwUnits = []
        for ii in range(1, self.nbRw+1):
            self.rwUnits.append(RwUnitParam(ii))
        self.isOn = False

    def updateRwSet(self, nbRw):
        self.nbRw = nbRw
        for ii in range(1, self.nbRw+1):
            self.rwUnits.append(RwUnitParam(ii))


# RW unit parameters
class RwUnitParam:
    def __init__(self, rwId):

        self.rwId = rwId
        self.posUnit_G = np.array([0, 0, 0]) # [-]
        self.dirUnit_G = np.array([0, 0, -1]) # [-]
        self.inertia = 0 # [kg*m^2]