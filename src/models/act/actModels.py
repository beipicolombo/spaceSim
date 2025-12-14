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
		# Interface
		self.interface = {"torqueAct_B": np.array([0, 0, 0])}
		self.interface.update({"thr": self.thrModel.interface})
		self.interface.update({"rw": self.rwModel.interface})

	def run(self, simOptions, fswIf):
		# Run RW model
		self.rwModel.run(simOptions, fswIf)
		# Run THR model
		self.thrModel.run(simOptions, fswIf)
		# Compute total actuators torque
		self.computeActTorque()

	def computeActTorque(self):
		# Fetch THR and RW interfaces
		torqueRw_B = self.thrModel.interface["torque_B"]
		torqueThr_B = self.rwModel.interface["torque_B"]
		# Process
		torqueAct_B = torqueThr_B + torqueRw_B
		# Update interface
		self.interface["torqueAct_B"] = torqueAct_B
