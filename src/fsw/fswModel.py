import numpy as np
import src.fsw.fswControl as fswControl
import src.fsw.fswEstimation as fswEstimation
import src.fsw.fswGuidance as fswGuidance
import src.fsw.fswCommand as fswCommand
import src.fsw.fswModeMgt as fswModeMgt

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Param:
	def __init__(self, scParam):
		# Estimation
		self.estParam = fswEstimation.Param()
		# Guidance
		self.guidParam = fswGuidance.Param()
		# Control
		self.ctrParam = fswControl.Param()
		# Command
		self.cmdParam = fswCommand.Param(scParam.actParam)
		# Common

		# Mode management
		self.modeMgtParam = fswModeMgt.Param()

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------