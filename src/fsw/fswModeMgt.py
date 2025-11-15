
import numpy as np
import src.interfaces.eventsAndTmTc as eventsAndTmTc

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# Dictionary from AOCS modes to IDs
AOCSMODES = {}
AOCSMODES["OFF"] = 0
AOCSMODES["SAFE"] = 1
AOCSMODES["NOM_PTNG"] = 2
AOCSMODES["NOM_EQ"] = 3
AOCSMODES["NOM_SLEW"] = 4
AOCSMODES["OCM"] = 5
AOCSMODES["MANUAL"] = 6

# Dictionary from IDs to AOCS modes
AOCSMODES_ID = {}
for key in AOCSMODES.keys():
	aocsModeNb = AOCSMODES[key]
	AOCSMODES_ID[aocsModeNb] = key

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class FswModeMgtParam:
	def __init__(self):
        # Initial AOCS mode
		self.aocsModeInit = AOCSMODES["OFF"]
		self.aocsOffModeMinDur = 10 # [s]
		# SAFE mode parameters
		self.aocsSafeModeMinDur = 0*60 # [s]
		self.aocsSafeModeAngRateThd = 0.02*deg2rad # [s]
		self.aocsSafeModeAngRateThdDur = 10*60 # [s]
		self.isAutoSafeToNomPtngModeAllwd = False
		# NOM_PTNG parameters
		self.aocsNomPtngModeMinDur = 30*60 # [s]
		self.isAutoNomPtngToNomEqAllwd = False
		# NOM_EQ parameters
		self.aocsNomEqModeMinDur = 90*60 # [s
		self.isAutoNomToOcmAllwd = False
		# NOM common parameters
		self.aocsNomModeAngRateThd = 0.01*deg2rad # [s]
		self.aocsNomModeAngRateThdDur = 10*60 # [s]


class FswModeMgtState:
	def __init__(self, aocsMode):
		aocsGuidMode = aocsGuidanceModeLogic(aocsMode)
		(aocsCtrMode, aocsCtrActMode) = aocsCtrModeLogic(aocsMode)

		# Previous states
		self.aocsModePre = aocsMode
		self.aocsCtrModePre = aocsCtrMode
		self.aocsGuidModePre = aocsGuidMode
		self.aocsCtrActModePre = aocsCtrActMode
		self.aocsModeElapsedTimePre = 0
		self.isRateBelowThdPre = False
		self.rateBelowThdCounterPre = 0

		# Current state
		self.aocsMode = aocsMode
		self.aocsCtrMode = aocsCtrMode
		self.aocsGuidMode = aocsGuidMode
		self.aocsCtrActMode = aocsCtrActMode
		self.aocsModeElapsedTime = 0
		self.angRateThd = 0
		self.angRateThdDur = 0


	def update(self, simParam, modeMgtParam, fswEstBus, simBus, receivedTcList):
		# Retrieve useful data
		Ts = simParam.Ts
		angRateEst_BI_B = fswEstBus.signals["angRateEst_BI_B"].value # from previous timestep	
		elapsedTime = simBus.signals["elapsedTime"].value

		# Initialize
		eventsList = []
		aocsMode = self.aocsMode
		isAocsModeTrans = False


		# 0. Check common conditions for transitions
		isRateBelowThd = (np.linalg.norm(angRateEst_BI_B) < self.angRateThd)
		isRateBelowThdCounterReset = (self.isRateBelowThdPre and ~isRateBelowThd)
		rateBelowThdCounter = runCounter(self.rateBelowThdCounterPre, isRateBelowThdCounterReset, Ts)
		if (isRateBelowThd and (rateBelowThdCounter >= self.angRateThdDur)):
			isRateCvg = True
		else:
			isRateCvg = False


		# 1. Retrieve TC
		(isTcFromSafeToNomPtngAccepted, isTcFromNomPtngToNomEqAccepted, eventsList) = tcMgt(self.aocsMode, isRateCvg, self.aocsModeElapsedTime, elapsedTime, receivedTcList, eventsList, modeMgtParam)


		# 2. Check AOCS modes transitions
		if ((AOCSMODES_ID[self.aocsMode] == "OFF") and (self.aocsModeElapsedTime >= modeMgtParam.aocsOffModeMinDur)):
			# Transition OFF -> SAFE
			aocsMode = AOCSMODES["SAFE"]
			eventsList.append(eventsAndTmTc.Event(name = "EVT_AOCS_MODE_SWITCH", id = 1, time = elapsedTime))

		elif (AOCSMODES_ID[self.aocsMode] == "SAFE"):
			# Check auto transition
			isAutoSafeToNomPtngOk = ((self.aocsModeElapsedTime > modeMgtParam.aocsSafeModeMinDur) and isRateCvg and modeMgtParam.isAutoSafeToNomPtngModeAllwd)

			if (isAutoSafeToNomPtngOk or isTcFromSafeToNomPtngAccepted):
				# Transition SAFE -> NOM_PTNG
				aocsMode = AOCSMODES["NOM_PTNG"]
				eventsList.append(eventsAndTmTc.Event(name = "EVT_AOCS_MODE_SWITCH", id = 1, time = elapsedTime))

		elif ((AOCSMODES_ID[self.aocsMode] == "NOM_PTNG") and (self.aocsModeElapsedTime > modeMgtParam.aocsNomPtngModeMinDur) and isRateCvg and modeMgtParam.isAutoNomPtngToNomEqAllwd):
			# Transition NOM -> OCM, TBW dummy for now
			# Check auto transition
			isAutoSafeNomPtngToNomEqOk = ((self.aocsModeElapsedTime > modeMgtParam.aocsNomPtngModeMinDur) and isRateCvg and modeMgtParam.isAutoSafeToNomPtngModeAllwd)

			if (isAutoSafeNomPtngToNomEqOk or isTcFromNomPtngToNomEqAccepted):
				aocsMode = AOCSMODES["NOM_EQ"]
				eventsList.append(eventsAndTmTc.Event(name = "EVT_AOCS_MODE_SWITCH", id = 1, time = elapsedTime))

		elif ((AOCSMODES_ID[self.aocsMode] == "NOM_EQ") and (self.aocsModeElapsedTime > modeMgtParam.aocsNomEqModeMinDur) and isRateCvg and modeMgtParam.isAutoNomToOcmAllwd):
			aocsMode = AOCSMODES["OCM"]
			eventsList.append(eventsAndTmTc.Event(name = "EVT_AOCS_MODE_SWITCH", id = 1, time = elapsedTime))

		# Detec if transition occured
		if aocsMode != self.aocsMode:
			isAocsModeTrans = True

		# 3. Set states specific to the current AOCS mode
		if (AOCSMODES_ID[aocsMode] == "SAFE"):
			angRateThd = modeMgtParam.aocsSafeModeAngRateThd
			angRateThdDur = modeMgtParam.aocsSafeModeAngRateThdDur
		elif ((AOCSMODES_ID[aocsMode] == "NOM_PTNG") or (AOCSMODES_ID[aocsMode] == "NOM_EQ")):
			angRateThd = modeMgtParam.aocsNomModeAngRateThd
			angRateThdDur = modeMgtParam.aocsNomModeAngRateThdDur
		else:
			angRateThd = 0
			angRateThdDur = 0

		# 4. Switch guidance  depending on current AOCS mode
		aocsGuidMode = aocsGuidanceModeLogic(aocsMode)

		# 5. Switch control and actuator mode depending on current AOCS mode
		(aocsCtrMode, aocsCtrActMode) = aocsCtrModeLogic(aocsMode)

		# 6. Compute elapsed times
		isAocsModeElapsedTimeCounterReset = isAocsModeTrans
		aocsModeElapsedTime = runCounter(self.aocsModeElapsedTime, isAocsModeElapsedTimeCounterReset, Ts)

		# 8. Update states
		self.aocsModePre = self.aocsMode
		self.aocsCtrModePre = self.aocsCtrMode
		self.aocsGuidModePre = self.aocsGuidMode
		self.aocsCtrActModePre = self.aocsCtrActMode
		self.aocsModeElapsedTimePre = self.aocsModeElapsedTime
		self.isRateBelowThdPre = isRateBelowThd
		self.rateBelowThdCounterPre = rateBelowThdCounter

		self.aocsMode = aocsMode
		self.aocsCtrMode = aocsCtrMode
		self.aocsCtrActMode = aocsCtrActMode
		self.aocsGuidMode = aocsGuidMode
		self.aocsModeElapsedTime = aocsModeElapsedTime
		self.angRateThd = angRateThd
		self.angRateThdDur = angRateThdDur

		return (eventsList, isAocsModeTrans)

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

def computeModeMgt(simParam, fswParam, fswModeMgtState, fswBus, simBus, receivedTcList):
	# Initialize output bus
	fswModeMgtBusOut = fswBus.subBuses["modeMgt"]
	elapsedTime = simBus.signals["elapsedTime"].value

	allEvents = []

	# Manage received TC
	for receivedTc in receivedTcList:
		allEvents.append(eventsAndTmTc.Event(name = "EVT_TC_RECEIVED", id = 2, time = receivedTc.time))

	# Update mode management states
	(eventsList, isAocsModeTrans) = fswModeMgtState.update(simParam, fswParam.modeMgtParam, fswBus.subBuses["estimation"], simBus, receivedTcList)
	allEvents = allEvents + eventsList 

	# Update signals
	fswBusOut = fswBus
	fswModeMgtBusOut.signals["aocsMode"].update(fswModeMgtState.aocsMode)
	fswModeMgtBusOut.signals["aocsGuidMode"].update(fswModeMgtState.aocsGuidMode)
	fswModeMgtBusOut.signals["aocsCtrMode"].update(fswModeMgtState.aocsCtrMode)
	fswModeMgtBusOut.signals["aocsCtrActMode"].update(fswModeMgtState.aocsCtrActMode)
	fswModeMgtBusOut.signals["aocsModeElapsedTime"].update(fswModeMgtState.aocsModeElapsedTime)
	fswModeMgtBusOut.signals["isAocsModeTrans"].update(isAocsModeTrans)

	# Manage events
	for event in allEvents:
		event.display()

	return (fswModeMgtBusOut, allEvents)
	

def aocsGuidanceModeLogic(aocsMode):
	if (AOCSMODES_ID[aocsMode] == "SAFE"):
		aocsGuidMode = "GUIDMODE_RATE_DAMPING"
	elif (AOCSMODES_ID[aocsMode] == "NOM_PTNG"):
		aocsGuidMode = "GUIDMODE_ATT_INERT" 
	elif (AOCSMODES_ID[aocsMode] == "NOM_EQ"):
		aocsGuidMode = "GUIDMODE_ATT_NADIR"
	elif (AOCSMODES_ID[aocsMode] == "OCM"):
		aocsGuidMode = "GUIDMODE_ATT_INERT"
	elif (AOCSMODES_ID[aocsMode] == "MANUAL"):
		aocsGuidMode = "GUIDMODE_OFF"
        # TBW: GUIDMODE_OFF for now
	else:
		aocsGuidMode = "GUIDMODE_OFF"

	return aocsGuidMode

def aocsCtrModeLogic(aocsMode):
	if (AOCSMODES_ID[aocsMode] == "SAFE"):
		aocsCtrMode = "CTRLMODE_RATE_DAMP_CTRL"
		aocsCtrActMode = "THR"
	elif (AOCSMODES_ID[aocsMode] == "NOM_PTNG"):
		aocsCtrMode = "CTRLMODE_ATT_CTRL"
		aocsCtrActMode = "THR"
	elif (AOCSMODES_ID[aocsMode] == "NOM_EQ"):
		aocsCtrMode = "CTRLMODE_ATT_CTRL"
		aocsCtrActMode = "THR"
	elif (AOCSMODES_ID[aocsMode] == "OCM"):
		aocsCtrMode = "CTRLMODE_THRUST_CTRL"
		aocsCtrActMode = "THR"
	elif (AOCSMODES_ID[aocsMode] == "MANUAL"):
		aocsCtrMode = "CTRLMOD_MANUAL"
		aocsCtrActMode = "THR"
	else:
		aocsCtrMode = "CTRLMODE_OFF"
		aocsCtrActMode = "NONE"

	return (aocsCtrMode, aocsCtrActMode)


def runCounter(state, isReset, Ts):
	if isReset:
		state = 0
	else:
		state = state + Ts
	return state


def tcMgt(aocsMode, isRateCvg, aocsModeElapsedTime, elapsedTime, receivedTcList, eventsListIn, modeMgtParam):
	
	# Initialize
	isTcFromSafeToNomPtngAccepted = False
	isTcFromNomPtngToNomEqAccepted = False
	eventsList = eventsListIn

	# Update the TC reception status
	if (len(receivedTcList) > 0):
		for receivedTc in receivedTcList:
			isTcFromSafeToNomPtngReceived = (receivedTc.name == "TC_AOCS_MODE_SWITCH_SAFE_TO_NOM_PTNG")
			isTcFromNomPtngToNomEqReceived = (receivedTc.name == "TC_AOCS_MODE_SWITCH_NOM_PTNG_TO_NOM_EQ")

			isTcFromSafeToNomPtngAccepted = (isTcFromSafeToNomPtngReceived and (AOCSMODES_ID[aocsMode] == "SAFE") and isRateCvg and (aocsModeElapsedTime > modeMgtParam.aocsSafeModeMinDur))
			isTcFromNomPtngToNomEqAccepted = (isTcFromNomPtngToNomEqReceived and (AOCSMODES_ID[aocsMode] == "NOM_PTNG") and isRateCvg and (aocsModeElapsedTime > modeMgtParam.aocsNomPtngModeMinDur))

			if isTcFromSafeToNomPtngAccepted:
				eventsList.append(eventsAndTmTc.Event(name = "EVT_TC_ACCEPTED", id = 3, time = elapsedTime))
			elif (isTcFromSafeToNomPtngReceived and not(isTcFromSafeToNomPtngAccepted)):
				eventsList.append(eventsAndTmTc.Event(name = "EVT_TC_REJECTED", id = 4, time = elapsedTime))

			if isTcFromNomPtngToNomEqAccepted:
				eventsList.append(eventsAndTmTc.Event(name = "EVT_TC_ACCEPTED", id = 3, time = elapsedTime))
			elif (isTcFromNomPtngToNomEqReceived and not(isTcFromNomPtngToNomEqAccepted)):
				eventsList.append(eventsAndTmTc.Event(name = "EVT_TC_REJECTED", id = 4, time = elapsedTime))

	return (isTcFromSafeToNomPtngAccepted, isTcFromNomPtngToNomEqAccepted, eventsList)

