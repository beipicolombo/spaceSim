import numpy as np


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class FswModeMgtState:
	# TBW
	def __init__(self, aocsMode, aocsGuidMode, aocsCtrMode, aocsCtrActMode):
		# Previous states
		self.aocsModePre = aocsMode
		self.aocsCtrModePre = aocsCtrMode
		self.aocsGuidModePre = aocsGuidMode
		self.aocsCtrActModePre = aocsCtrActMode
		self.aocsModeElapsedTimePre = 0

		# Current state
		self.aocsMode = aocsMode
		self.aocsCtrMode = aocsCtrMode
		self.aocsGuidMode = aocsGuidMode
		self.aocsCtrActMode = aocsCtrActMode
		self.aocsModeElapsedTime = 0

	def update(self, simParam):
		# Retrieve useful data
		Ts = simParam.Ts

		# Compute new AOCS mode => TBW currently constant
		aocsMode = self.aocsMode
		# Compute new guidance mode => TBW currently constant
		aocsGuidMode = self.aocsGuidMode
		# Compute new control mode => TBW currently constant
		aocsCtrMode = self.aocsCtrMode
		aocsCtrActMode = self.aocsCtrActMode
		
		# Compute elapsed time
		if (aocsCtrMode != self.aocsCtrModePre):
			aocsModeElapsedTime = 0
		else:
			aocsModeElapsedTime = self.aocsModeElapsedTimePre + Ts

		# Update states
		self.aocsModePre = self.aocsMode
		self.aocsCtrModePre = self.aocsCtrMode
		self.aocsGuidModePre = self.aocsGuidMode
		self.aocsCtrActModePre = self.aocsCtrActMode
		self.aocsModeElapsedTimePre = self.aocsModeElapsedTime

		self.aocsMode = aocsMode
		self.aocsCtrMode = aocsCtrMode
		self.aocsCtrActMode = aocsCtrActMode
		self.aocsGuidMode = aocsGuidMode
		self.aocsModeElapsedTime = aocsModeElapsedTime

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def computeModeMgt(simParam, fswModeMgtState, fswBus):
	# Update mode management states
	fswModeMgtState.update(simParam)

	fswModeMgtBusOut = fswBus.subBuses["modeMgt"]

	# Update signals
	fswBusOut = fswBus
	fswModeMgtBusOut.signals["aocsMode"].update(fswModeMgtState.aocsMode)
	fswModeMgtBusOut.signals["aocsGuidMode"].update(fswModeMgtState.aocsGuidMode)
	fswModeMgtBusOut.signals["aocsCtrMode"].update(fswModeMgtState.aocsCtrMode)
	fswModeMgtBusOut.signals["aocsCtrActMode"].update(fswModeMgtState.aocsCtrActMode)
	fswModeMgtBusOut.signals["aocsModeElapsedTime"].update(fswModeMgtState.aocsModeElapsedTime)

	return fswModeMgtBusOut

