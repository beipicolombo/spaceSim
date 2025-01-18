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
class ImuModelParam():
	def __init__(self):
		# TBW
		self.dummy = 0

	def computeAngRateMeasurement(self, modelsBus):
		angRate_BI_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].value
		angRateMeas_B = angRate_BI_B
		return (angRateMeas_B)


