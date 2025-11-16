import numpy as np
import src.models.act.thrModels as thrModels
import src.models.act.rwModels as rwModels


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Param:
	# [TBW] For now the commanding function has perfect knowledge of the model parameters
	# The parameters structure is exactly the same
	def __init__(self, scActParam):
		self.rwCmdParam = scActParam.thrModel
		self.thrCmdParam = scActParam.rwModel


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def computeCommand(fswCmdParam, fswBus):
	fswBusOut = fswBus

	# Run THR command function
	fswBusOut = thrCommand(fswCmdParam.thrCmdParam, fswBusOut)
	# Run RW command function
	fswBusOut = rwCommand(fswCmdParam.rwCmdParam, fswBusOut)

	return fswBusOut.subBuses["command"]


# Dummy function for THR command => TBW
def thrCommand(thrFswParam, fswBus):
	fswBusOut = fswBus

	torqueCmdThr_B = fswBus.subBuses["control"].signals["torqueCtrlThr_B"].value
	fswBusOut.subBuses["command"].signals["torqueCmdThr_B"].update(torqueCmdThr_B)
	return fswBusOut


# Dummy function for RW command => TBW
def rwCommand(rwFswParam, fswBus):
	fswBusOut = fswBus

	torqueCmdRw_B = fswBus.subBuses["control"].signals["torqueCtrlRw_B"].value
	fswBusOut.subBuses["command"].signals["torqueCmdRw_B"].update(torqueCmdRw_B)
	return fswBusOut

