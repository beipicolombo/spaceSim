import numpy as np

import src.utils.paramTools as paramTools

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


	def set(self, patches, models):
		# Set high level FSW parameters
		# FSW has full knowledge of actuator model => TBW to be descoped
		self.param.inertiaSc_B = models.spacecraft.massModel.inertiaSc_B
		self.param.inertiaSc_G = models.spacecraft.massModel.inertiaSc_G
		self.param.massCB = models.spacecraft.massModel.massCB
		self.param.massSc = models.spacecraft.massModel.massSc
		self.param.comSc_B = models.spacecraft.massModel.comSc_B

		# [Mode management] Patch simulation-specific parameters and update state 
		self.modeMgt.param = paramTools.patchAttributes(self.modeMgt.param, patches["modeMgtParamPatch"])
		self.modeMgt.state = fswModeMgt.State(self.modeMgt.param.aocsModeInit)

		# [Guidance] Patch simulation-specific parameters
		self.guidance.param = paramTools.patchAttributes(self.guidance.param, patches["guidParamPatch"])

		# [Control] Patch simulation-specific parameters
		# FSW has full knowledge of spacecraft mass model
		self.control.param = paramTools.patchAttributes(self.control.param, patches["ctrParamPatch"])
		self.control.set(self.param)
		
		# [Command] TBW
		# FSW has full knowledge of actuator model => TBW to be descoped


	def run(self, simulation, modelsIf):
		# Fetch input
		simIf = simulation.interface
		simParam = simulation.param

		self.param = 0
		# Fetch interface
		fswEstIfPre = self.interface["estimation"] # From previous timestep

		# Run mode management
		# Received TC list to implemented in simIf
		self.modeMgt.run(simParam, simIf, fswEstIfPre)
		fswModeMgtIf = self.interface["modeMgt"] # Current timestep
		
		# Run sensor processing
		self.sensorProc.run(fswModeMgtIf)
		fswSensorProcIf = self.interface["sensorProc"] # Current timestep

		# Run estimation
		self.estimation.run(fswModeMgtIf, fswSensorProcIf, modelsIf)
		fswEstIf = self.interface["estimation"] # Current timestep

		# Run guidance
		self.guidance.run(fswModeMgtIf, fswEstIf)
		fswGuidIf = self.interface["guidance"] # Current timestep

		# Run control
		self.control.run(fswModeMgtIf, fswEstIf, fswGuidIf)
		fswCtrlIf = self.interface["control"] # Current timestep

		# Run command
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