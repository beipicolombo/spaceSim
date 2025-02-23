import numpy as np


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class FswModeMgtState:
	# TBW
	def __init__(self, fswParam):
		# Previous states
		self.aocsModePre = "OFF"
		self.aocsCtrModePre = "CTRLMODE_OFF"
		self.aocsGuidModePre = "GUIDMODE_OFF"
		self.aocsCtrActModePre = "NONE"

		# Current state
		self.aocsMode = "OFF"
		self.aocsCtrMode = "CTRLMODE_OFF"
		self.aocsGuidMode = "GUIDMODE_OFF"
		self.aocsCtrActMode = "NONE"

	def update(self, fswParam):
		self.aocsModePre = self.aocsMode
		self.aocsCtrModePre = self.aocsCtrMode
		self.aocsGuidModePre = self.aocsGuidMode
		self.aocsCtrActModePre = self.aocsCtrActMode

		# Compute new AOCS mode
		aocsMode = "OFF"

		# Compute new guidance mode
		aocsGuidMode = fswParam.guidParam.MODE

		# Compute new control mode
		aocsCtrMode = fswParam.ctrParam.MODE
		aocsCtrActMode = fswParam.ctrParam.ACTMODE

		# Update states
		self.aocsMode = aocsMode
		self.aocsCtrMode = aocsCtrMode
		self.aocsCtrActMode = aocsCtrActMode
		self.aocsGuidMode = aocsGuidMode

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def computeModeMgt(fswParam, fswModeMgtState, fswBus):
	# Update mode management states
	fswModeMgtState.update(fswParam)

	fswModeMgtBusOut = fswBus.subBuses["modeMgt"]

	# Update signals
	fswBusOut = fswBus
	fswModeMgtBusOut.signals["aocsMode"].update(fswModeMgtState.aocsMode)
	fswModeMgtBusOut.signals["aocsGuidMode"].update(fswModeMgtState.aocsGuidMode)
	fswModeMgtBusOut.signals["aocsCtrMode"].update(fswModeMgtState.aocsCtrMode)
	fswModeMgtBusOut.signals["aocsCtrActMode"].update(fswModeMgtState.aocsCtrActMode)

	return fswModeMgtBusOut

