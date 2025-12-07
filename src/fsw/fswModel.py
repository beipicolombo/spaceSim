import numpy as np
import src.fsw.fswModeMgt as fswModeMgt
import src.fsw.fswSensorProc as fswSensorProc
import src.fsw.fswEstimation as fswEstimation
import src.fsw.fswGuidance as fswGuidance
import src.fsw.fswControl as fswControl
import src.fsw.fswCommand as fswCommand

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Fsw:
	def __init__(self):
		self.param = 0
		self.modeMgt = fswModeMgt.ModeMgt()
		self.sensorProc = fswSensorProc.SensorProcessing()
		self.estimation = fswEstimation.Estimation()
		self.guidance = fswGuidance.Guidance()
		self.control = fswControl.Control()
		self.command = fswCommand.Command()

class Param:
	def __init__(self, scParam):
		# Common
		# ...
		# Mode management
		self.modeMgtParam = fswModeMgt.Param()
		# Estimation
		self.estParam = fswEstimation.Param()
		# Guidance
		self.guidParam = fswGuidance.Param()
		# Control
		self.ctrParam = fswControl.Param()
		# Command
		self.cmdParam = fswCommand.Param(scParam.act)


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------