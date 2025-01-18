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
class StrModelParam():
	def __init__(self):
		# TBW
		self.dummy = 0

	def computeAttitudeMeasurement(self, modelsBus):
		eulerAng_BI = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].value
		eulerAngMeas_BI = eulerAng_BI
		return (eulerAngMeas_BI)

