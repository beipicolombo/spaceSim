import numpy as np
import src.models.act.actModels as actModels
import src.models.sen.senModels as senModels
import src.models.dyn.massModels as massModels

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Spacecraft:
	def __init__(self):
		# Mass param
		self.massParam = massModels.MassParam()
		# Sensors
		self.senParam = senModels.SenModelParam()
		# Actuators
		self.actParam = actModels.ActModelParam(self.massParam)

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------