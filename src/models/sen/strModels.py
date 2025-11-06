# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np
import src.models.kin.attitudeKinematics as attitudeKinematics
# import math as m


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
		qBI_sca     = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].value
		qBI_vec     = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].value
		eulerAng_BI = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].value
		eulerAngMeas_BI = eulerAng_BI
		qBImeas_sca = qBI_sca
		qBImeas_vec = qBI_vec
		qBImeas = attitudeKinematics.Quaternion(qBImeas_sca, qBImeas_vec).normalize()
		return (qBImeas, eulerAngMeas_BI)

