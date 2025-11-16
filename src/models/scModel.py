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
class Spacecraft:
	def __init__(self, massModelName = "Default"):
		# Template
		self.massModelName = massModelName
		# Mass param
		self.massParam = scMassModelsDb.getSpaceceraftMassModel(massModelName = self.massModelName)
		# Sensors
		self.senParam = senModels.Param()
		# Actuators
		self.actParam = actModels.Param(self.massParam)

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------