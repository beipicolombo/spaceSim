
import numpy as np

import src.models.scModel as scModel


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# MODELS
# --------------------------------------------------
class Models():
	def __init__(self, massModelName, allParam):
		self.spacecraft = scModel.Spacecraft(massModelName, allParam)
