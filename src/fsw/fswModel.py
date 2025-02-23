import numpy as np
import src.fsw.fswControl as fswControl
import src.fsw.fswEstimation as fswEstimation
import src.fsw.fswGuidance as fswGuidance
import src.fsw.fswCommand as fswCommand

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Fsw:
	def __init__(self, scParam):
		# Estimation
		self.estParam = fswEstimation.FswEstimationParam()
		# Guidance
		self.guidParam = fswGuidance.FswGuidanceParam()
		# Control
		self.ctrParam = fswControl.FswControlParam()
		# Command
		self.cmdParam = fswCommand.FswCommandParam(scParam.actParam)
		# Common

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------