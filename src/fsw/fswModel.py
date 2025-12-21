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
		# Command parameters
		self.param = Param()
		# FSW function
		self.modeMgt = fswModeMgt.ModeMgt()
		self.sensorProc = fswSensorProc.SensorProcessing()
		self.estimation = fswEstimation.Estimation()
		self.guidance = fswGuidance.Guidance()
		self.control = fswControl.Control()
		self.command = fswCommand.Command()
		# Interface
		self.interface = {}
		self.interface.update({"modeMgt": self.modeMgt.interface})
		self.interface.update({"sensorProc": self.sensorProc.interface})
		self.interface.update({"estimation": self.estimation.interface})
		self.interface.update({"guidance": self.guidance.interface})
		self.interface.update({"control": self.control.interface})
		self.interface.update({"command": self.command.interface})


	def build(self, patches, models):
		# [Common] High level FSW parameters
		# [TBW] For now FSW has full knowledge of spacecraft mass model
		self.param.inertiaSc_B = models.spacecraft.massModel.inertiaSc_B
		self.param.inertiaSc_G = models.spacecraft.massModel.inertiaSc_G
		self.param.massCB = models.spacecraft.massModel.massCB
		self.param.massSc = models.spacecraft.massModel.massSc
		self.param.comSc_B = models.spacecraft.massModel.comSc_B

		# [Mode management]
		self.modeMgt.build(patches)

		# [Sensor processing]
		self.sensorProc.build(patches)

		# [Estimation]
		self.estimation.build(patches)

		# [Guidance]
		self.guidance.build(patches)

		# [Control]
		self.control.build(patches, self.param)
		
		# [Command]
		self.command.build(patches)


	def run(self, simulation, modelsIf):
		# Fetch input
		simIf = simulation.interface
		simParam = simulation.param

		self.param = 0
		# Fetch interface
		fswEstIfPre = self.interface["estimation"] # From previous timestep

		# [Mode management}
		self.modeMgt.run(simParam, simIf, fswEstIfPre)
		fswModeMgtIf = self.interface["modeMgt"] # Current timestep
		
		# [Sensor processing]
		self.sensorProc.run(fswModeMgtIf)
		fswSensorProcIf = self.interface["sensorProc"] # Current timestep

		# [Estimation]
		self.estimation.run(fswModeMgtIf, fswSensorProcIf, modelsIf)
		fswEstIf = self.interface["estimation"] # Current timestep

		# [Guidance]
		self.guidance.run(fswModeMgtIf, fswEstIf)
		fswGuidIf = self.interface["guidance"] # Current timestep

		# [Control]
		self.control.run(fswModeMgtIf, fswEstIf, fswGuidIf)
		fswCtrlIf = self.interface["control"] # Current timestep

		# [Command]
		self.command.run(fswModeMgtIf, fswCtrlIf)


# High level FSW parameters
class Param:
	def __init__(self):
		# Spacecraft mass model
		self.inertiaSc_B = np.identity(3)
		self.inertiaSc_G = np.identity(3)
		self.massCB = 0
		self.massSc = 0
		self.comSc_B = np.array([0, 0, 0])

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------