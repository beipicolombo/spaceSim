# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np
import src.models.sen.strModels as strModels
import src.models.sen.imuModels as imuModels
# import math as m
# import src.attitudeKinematics as attitudeKinematics

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Sensors:
	def __init__(self, senParam):
		self.str = strModels.Str(senParam["str"])
		self.imu = imuModels.Imu(senParam["imu"])
		# Interface
		self.interface = {}
		self.interface.update({"imu": self.imu.interface})
		self.interface.update({"str": self.str.interface})

	def run(self, fswIf, dynIf, envIf):
		# Process
		self.imu.run(dynIf, envIf)
		self.str.run(dynIf, envIf)

