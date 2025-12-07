
import numpy as np
import src.models.act.thrModels as thrModels
import src.models.act.rwModels as rwModels


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Command():
	def __init__(self):
		self.param = 0


	def run(self, fswBus):
		fswCmdParam = self.param
		fswBusOut = fswBus
		# Run THR command function
		fswBusOut = self.thrCommand(fswBusOut)
		# Run RW command function
		fswBusOut = self.rwCommand(fswBusOut)
		return fswBusOut.subBuses["command"]


	# Dummy function for THR command => TBW
	def thrCommand(self, fswBus):
		fswBusOut = fswBus
		torqueCmdThr_B = fswBus.subBuses["control"].signals["torqueCtrlThr_B"].value
		fswBusOut.subBuses["command"].signals["torqueCmdThr_B"].update(torqueCmdThr_B)
		return fswBusOut


	# Dummy function for RW command => TBW
	def rwCommand(self, fswBus):
		fswBusOut = fswBus
		torqueCmdRw_B = fswBus.subBuses["control"].signals["torqueCtrlRw_B"].value
		fswBusOut.subBuses["command"].signals["torqueCmdRw_B"].update(torqueCmdRw_B)
		return fswBusOut


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------