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
class Actuators:
	def __init__(self, attDynParam, actParam):
		self.thrModel = thrModels.Thr(actParam["thr"])
		self.rwModel = rwModels.Rw(actParam["rw"])

	def run(self, simOptions, fswCmdBus, modelsActBus):
		# Initialize outputs bus
		modelsActBusOut = modelsActBus
		# Run RW model
		modelsActBusOut = self.rwModel.run(simOptions, fswCmdBus, modelsActBusOut)
		# Run THR model
		modelsActBusOut = self.thrModel.run(simOptions, fswCmdBus, modelsActBusOut)
		# Compute total actuators torque
		modelsActBusOut = self.computeActTorque(modelsActBusOut)
		return modelsActBus


	def computeActTorque(self, modelsActBus):
		# Retrieve useful inputs
		torqueRw_B = modelsActBus.signals["torqueRw_B"].value
		torqueThr_B = modelsActBus.signals["torqueThr_B"].value
		# Initialize outputs bus
		modelsActBusOut = modelsActBus
		# Process
		torqueAct_B = torqueThr_B + torqueRw_B
		# Set outputs bus
		modelsActBusOut.signals["torqueAct_B"].update(torqueAct_B)
		return modelsActBusOut
