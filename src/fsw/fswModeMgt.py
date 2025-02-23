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

		# Current state
		self.aocsMode = aocsMode
		self.aocsCtrMode = aocsCtrMode
		self.aocsGuidMode = aocsGuidMode
		self.aocsCtrActMode = aocsCtrActMode

	def update(self):
		self.aocsModePre = self.aocsMode
		self.aocsCtrModePre = self.aocsCtrMode
		self.aocsGuidModePre = self.aocsGuidMode
		self.aocsCtrActModePre = self.aocsCtrActMode

		# Compute new AOCS mode => TBW currently constant
		aocsMode = self.aocsMode

		# Compute new guidance mode => TBW currently constant
		aocsGuidMode = self.aocsGuidMode

		# Compute new control mode => TBW currently constant
		aocsCtrMode = self.aocsCtrMode
		aocsCtrActMode = self.aocsCtrActMode

		# Update states
		self.aocsMode = aocsMode
		self.aocsCtrMode = aocsCtrMode
		self.aocsCtrActMode = aocsCtrActMode
		self.aocsGuidMode = aocsGuidMode

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def computeModeMgt(fswModeMgtState, fswBus):
	# Update mode management states
	fswModeMgtState.update()

	fswModeMgtBusOut = fswBus.subBuses["modeMgt"]

	# Update signals
	fswBusOut = fswBus
	fswModeMgtBusOut.signals["aocsMode"].update(fswModeMgtState.aocsMode)
	fswModeMgtBusOut.signals["aocsGuidMode"].update(fswModeMgtState.aocsGuidMode)
	fswModeMgtBusOut.signals["aocsCtrMode"].update(fswModeMgtState.aocsCtrMode)
	fswModeMgtBusOut.signals["aocsCtrActMode"].update(fswModeMgtState.aocsCtrActMode)

	return fswModeMgtBusOut

