import numpy as np
import src.fsw.fswControl as fswControl
import src.fsw.fswEstimation as fswEstimation
import src.fsw.fswGuidance as fswGuidance

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Fsw:
	def __init__(self):
		# Mass param
		self.estParam = fswEstimation.FswEstimationParam()
		# Sensors
		self.guidParam = fswGuidance.FswGuidanceParam()
		# Actuators
		self.ctrParam = fswControl.FswControlParam()

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------