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
		# TBW
		self.param = 0
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

	def run(self, simulation, modelsIf, receivedTcList):
		# Fetch input
		simIf = simulation.interface
		simParam = simulation.param

		self.param = 0
		# Fetch interface
		fswEstIfPre = self.interface["estimation"] # From previous timestep

		# Run mode management
		# Received TC list to implemented in simIf
		self.modeMgt.run(simParam, simIf, fswEstIfPre, receivedTcList)
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
		fswCmdIf = self.interface["command"] # Current timestep


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------