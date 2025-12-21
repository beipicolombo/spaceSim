
import numpy as np

import src.utils.paramTools as paramTools

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
# Sensors model
class Sensors:
	def __init__(self, senParam):
		self.str = strModels.Str(senParam["str"])
		self.imu = imuModels.Imu(senParam["imu"])
		# Interface
		self.interface = {}
		self.interface.update({"imu": self.imu.interface})
		self.interface.update({"str": self.str.interface})

	# Build
	def build(self, patches):
		# Build objects
		self.str.build(patches)
		self.imu.build(patches)

	# Run
	def run(self, fswIf, dynIf, envIf):
		# Process
		self.imu.run(dynIf, envIf)
		self.str.run(dynIf, envIf)

