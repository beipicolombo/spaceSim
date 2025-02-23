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

		self.aocsOffModeMinDur = 10 # [s] => TBW manage mode management parameters separately
		self.aocsSafeModeMinDur = 20*60 # [s] => TBW manage mode management parameters separately

	def update(self, simParam):
		# Retrieve useful data
		Ts = simParam.Ts

		# 1. Compute new AOCS mode
		if ((self.aocsMode == "OFF") and (self.aocsModeElapsedTime > self.aocsOffModeMinDur)):
			# Transition OFF -> SAFE
			aocsMode = "SAFE"
		elif ((self.aocsMode == "SAFE") and (self.aocsModeElapsedTime > self.aocsSafeModeMinDur)):
			aocsMode = "NOM"
		else: 
			# Otherwise in current mode
			aocsMode = self.aocsMode


		# 2. Switch control, guidance and actuators depending on current AOCS mode
		if (aocsMode == "SAFE"):
			aocsGuidMode = "GUIDMODE_RATE_DAMPING"
			aocsCtrMode = "CTRLMODE_RATE_DAMP_CTRL"
			aocsCtrActMode = "THR"
		elif (aocsMode == "NOM"):
			aocsGuidMode = "GUIDMODE_ATT_INERT" 
			aocsCtrMode = "CTRLMODE_ATT_CTRL"
			aocsCtrActMode = "THR"
		else:
			aocsGuidMode = "GUIDMODE_OFF"
			aocsCtrMode = "CTRLMODE_OFF"
			aocsCtrActMode = "NONE"

		# 3. Compute elapsed time
		if (aocsCtrMode != self.aocsCtrModePre):
			aocsModeElapsedTime = 0
		else:
			aocsModeElapsedTime = self.aocsModeElapsedTimePre + Ts

		# 4. Update states
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

