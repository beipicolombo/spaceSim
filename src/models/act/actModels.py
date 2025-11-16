# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np
import src.models.act.thrModels as thrModels
import src.models.act.rwModels as rwModels
# import math as m
# import src.attitudeKinematics as attitudeKinematics

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Param:
	def __init__(self, attDynParam):
		self.thrModelParam = thrModels.Param(attDynParam)
		self.rwModelParam = rwModels.Param()

	def computeActTorque(self, simOptions, fswCmdBus, modelsActBus):
		torqueAct_B = np.array([0, 0, 0])
		torqueThr_B = np.array([0, 0, 0])
		torqueRw_B = np.array([0, 0, 0])

		if simOptions.isRwAct:
			torqueRw_B = self.rwModelParam.computeRwTorque()
		elif simOptions.isThrAct:
			torqueThr_B = self.thrModelParam.computeThrTorque(fswCmdBus)
		
		torqueAct_B = torqueThr_B + torqueRw_B

		modelsActBus.signals["torqueRw_B"].update(torqueRw_B)
		modelsActBus.signals["torqueThr_B"].update(torqueThr_B)
		modelsActBus.signals["torqueAct_B"].update(torqueAct_B)

		return modelsActBus
