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

	def computeMeasurements(self, modelsSenBus, modelsBus):
		angRateMeas_BI_B = self.imu.computeAngRateMeasurement(modelsBus)
		(qBImeas, eulerAngMeas_BI) = self.str.computeAttitudeMeasurement(modelsBus)

		modelsSenBus.signals["angRateMeas_BI_B"].update(angRateMeas_BI_B)
		modelsSenBus.signals["eulerAngMeas_BI"].update(eulerAngMeas_BI)
		modelsSenBus.signals["qBImeas_sca"].update(qBImeas.sca)
		modelsSenBus.signals["qBImeas_vec"].update(qBImeas.vec)

		return modelsSenBus

