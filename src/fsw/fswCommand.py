
import numpy as np
import src.models.act.thrModels as thrModels
import src.models.act.rwModels as rwModels


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Command():
	def __init__(self):
		self.param = 0
		# Interface
		self.interface = {"torqueCmdThr_B": np.array([0, 0, 0]), 
					      "torqueCmdRw_B": np.array([0, 0, 0])}

	def run(self, fswModeMgtIf, fswCtrlIf):
		# Retrieve parameters
		fswCmdParam = self.param
		# Run THR command function
		self.thrCommand(fswModeMgtIf, fswCtrlIf)
		# Run RW command function
		self.rwCommand(fswModeMgtIf, fswCtrlIf)
	
	# Dummy function for THR command => TBW
	def thrCommand(self, fswModeMgtIf, fswCtrlIf):
		# Fetch interface
		torqueCtrlThr_B = fswCtrlIf["torqueCtrlThr_B"]
		forceCtrlThr_B = fswCtrlIf["forceCtrlThr_B"]
		# Process
		torqueCmdThr_B = torqueCtrlThr_B
		# Update interface
		self.interface["torqueCmdThr_B"] = torqueCmdThr_B

	# Dummy function for RW command => TBW
	def rwCommand(self, fswModeMgtIf, fswCtrlIf):
		# Fetch interface
		torqueCtrlRw_B = fswCtrlIf["torqueCtrlRw_B"]
		# Process
		torqueCmdRw_B = torqueCtrlRw_B
		# Update interface
		self.interface["torqueCmdRw_B"] = torqueCmdRw_B


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------