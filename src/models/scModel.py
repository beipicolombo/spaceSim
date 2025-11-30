
from dataclasses import dataclass

import numpy as np

import src.models.act.actModels as actModels
import src.models.sen.senModels as senModels
import src.models.dyn.massModels as massModels
import src.data.spacecraftMassModelsDatabase as scMassModelsDb


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# ...

# --------------------------------------------------
# DATA CLASSES
# --------------------------------------------------
# Spacecraft mass model parameters
@dataclass
class MassParam:
	mass: float
	diagInertia: list
	crossInertia: list
	com_B: list

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class MassModel:
	def __init__(self, massParam):
		self.param = massParam
		self.build()

	def build(self):
		mass = self.param
		diagInertia = self.param.diagInertia
		crossInertia = self.param.crossInertia
		com_B = self.param.com_B
		# Build the inertia matrix in geometry frame
		inertiaSc_G = np.diag(diagInertia)
		inertiaSc_G[0, 1] = -crossInertia[0]
		inertiaSc_G[0, 2] = -crossInertia[0]
		inertiaSc_G[1, 2] = -crossInertia[2]
		inertiaSc_G[1, 0] = -crossInertia[0]
		inertiaSc_G[2, 0] = -crossInertia[0]
		inertiaSc_G[2, 1] = -crossInertia[2]
		# Build the inertia matrix in body frame [TBW]
		inertiaSc_B = inertiaSc_G
		# Compute the inertia matrix inverse once
		inertiaScInv_B = np.linalg.inv(inertiaSc_B)
		# Compute the spacecraft and central body mass [TBW]
		massSc = mass # [WIP]
		massCB = massSc # [WIP]
		# Set attributes
		self.inertiaSc_B = inertiaSc_B
		self.inertiaSc_G = inertiaSc_G
		self.inertiaScInv_B = inertiaScInv_B
		self.massCB = massCB
		self.masssc = massSc	

class Spacecraft:
	def __init__(self, massModelName, allParam):
		# Template
		self.massModelName = massModelName
		# Mass param [TBW]
		rawMassParam = MassParam(**allParam["spacecraft"]["Default"])
		self.massModel = MassModel(rawMassParam)
		# Sensors
		self.sen = senModels.Sensors(allParam["sen"])
		# Actuators
		self.act = actModels.Actuators(self.massModel, allParam["act"])

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------