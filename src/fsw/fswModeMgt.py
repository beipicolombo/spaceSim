
import numpy as np
import src.interfaces.eventsAndTmTc as eventsAndTmTc

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class FswModeMgtParam:
	def __init__(self):
        # Initial AOCS mode
        # OFF (Off)
        # SAFE (Safe)
        # NOM_PTNG (Nominal - pointing)
        # NOM_EQ (Nominal - equilibrium)
        # NOM_SLEW (Nominal - slew) => To be added
        # OCM (Orbit Control Mode)
        # MANUAL
		self.aocsModeInit = "OFF"
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
		isTcFromSafeToNomPtngReceived = False
		isTcFromNomPtngToNomEqReceived = False
		eventsList = []

		# 0. Retrieve TC
		if (len(receivedTcList) > 0):
			receivedTc = receivedTcList[0]
			isTcFromSafeToNomPtngReceived = (receivedTc.name == "TC_AOCS_MODE_SWITCH_SAFE_TO_NOM_PTNG")
			isTcFromNomPtngToNomEqReceived = (receivedTc.name == "TC_AOCS_MODE_SWITCH_NOM_PTNG_TO_NOM_EQ")

		# 1. Check angular rates convergence
		isRateBelowThd = (np.linalg.norm(angRateEst_BI_B) < self.angRateThd)
		isRateBelowThdCounterReset = (self.isRateBelowThdPre and ~isRateBelowThd)
		rateBelowThdCounter = runCounter(self.rateBelowThdCounterPre, isRateBelowThdCounterReset, Ts)
		if (isRateBelowThd and (rateBelowThdCounter >= self.angRateThdDur)):
			isRateCvg = True
		else:
			isRateCvg = False

		# 2. Check AOCS modes transitions
		if ((self.aocsMode == "OFF") and (self.aocsModeElapsedTime >= modeMgtParam.aocsOffModeMinDur)):
			# Transition OFF -> SAFE
			aocsMode = "SAFE"
			eventsList.append(eventsAndTmTc.Event(name = "EVT_AOCS_MODE_SWITCH", id = 1, time = elapsedTime))
		elif (self.aocsMode == "SAFE"):
			isCommonCondOk = (self.aocsModeElapsedTime > modeMgtParam.aocsSafeModeMinDur) and isRateCvg
			# Check auto transition
			isAutoSafeToNomPtngOk = (isCommonCondOk and modeMgtParam.isAutoSafeToNomPtngModeAllwd)
			# Check TC transition
			isTcFromSafeToNomPtngAccepted = False
			if isTcFromSafeToNomPtngReceived:
				isTcFromSafeToNomPtngAccepted = isCommonCondOk
				if isTcFromSafeToNomPtngAccepted:
					eventsList.append(eventsAndTmTc.Event(name = "EVT_TC_ACCEPTED", id = 3, time = elapsedTime))
				else:
					eventsList.append(eventsAndTmTc.Event(name = "EVT_TC_REJECTED", id = 4, time = elapsedTime))
			isTcSafeToNomPtngOk = isTcFromSafeToNomPtngReceived and isTcFromSafeToNomPtngAccepted
		
			if (isAutoSafeToNomPtngOk or isTcSafeToNomPtngOk):
				# Transition SAFE -> NOM_PTNG
				aocsMode = "NOM_PTNG"
				eventsList.append(eventsAndTmTc.Event(name = "EVT_AOCS_MODE_SWITCH", id = 1, time = elapsedTime))
			else:
				# Otherwise in current mode
				aocsMode = self.aocsMode
		elif ((self.aocsMode == "NOM_PTNG") and (self.aocsModeElapsedTime > modeMgtParam.aocsNomPtngModeMinDur) and isRateCvg and modeMgtParam.isAutoNomPtngToNomEqAllwd):
			# Transition NOM -> OCM, TBW dummy for now
			aocsMode = "NOM_EQ"
			eventsList.append(eventsAndTmTc.Event(name = "EVT_AOCS_MODE_SWITCH", id = 1, time = elapsedTime))
		elif ((self.aocsMode == "NOM_EQ") and (self.aocsModeElapsedTime > modeMgtParam.aocsNomEqModeMinDur) and isRateCvg and modeMgtParam.isAutoNomToOcmAllwd):
			aocsMode = "OCM"
			eventsList.append(eventsAndTmTc.Event(name = "EVT_AOCS_MODE_SWITCH", id = 1, time = elapsedTime))
		else: 
			# Otherwise in current mode
			aocsMode = self.aocsMode

		# 3. Set states specific to the current AOCS mode
		if (aocsMode == "SAFE"):
			angRateThd = modeMgtParam.aocsSafeModeAngRateThd
			angRateThdDur = modeMgtParam.aocsSafeModeAngRateThdDur
		elif ((aocsMode == "NOM_PTNG") or (aocsMode == "NOM_EQ")):
			angRateThd = modeMgtParam.aocsNomModeAngRateThd
			angRateThdDur = modeMgtParam.aocsNomModeAngRateThdDur
		else:
			angRateThd = 0
			angRateThdDur = 0


		# 4. Switch guidance  depending on current AOCS mode
		aocsGuidMode = aocsGuidanceModeLogic(aocsMode)

		# 5. Switch control and actuator mode depending on current AOCS mode
		(aocsCtrMode, aocsCtrActMode) = aocsCtrModeLogic(aocsMode)

		# 4. Compute elapsed times
		isAocsModeElapsedTimeCounterReset = (aocsCtrMode != self.aocsCtrModePre)
		aocsModeElapsedTime = runCounter(self.aocsModeElapsedTime, isAocsModeElapsedTimeCounterReset, Ts)

		# 6. Update states
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

		return eventsList

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
	allEvents = allEvents + fswModeMgtState.update(simParam, fswParam.modeMgtParam, fswBus.subBuses["estimation"], simBus, receivedTcList)

	# Update signals
	fswBusOut = fswBus
	fswModeMgtBusOut.signals["aocsMode"].update(fswModeMgtState.aocsMode)
	fswModeMgtBusOut.signals["aocsGuidMode"].update(fswModeMgtState.aocsGuidMode)
	fswModeMgtBusOut.signals["aocsCtrMode"].update(fswModeMgtState.aocsCtrMode)
	fswModeMgtBusOut.signals["aocsCtrActMode"].update(fswModeMgtState.aocsCtrActMode)
	fswModeMgtBusOut.signals["aocsModeElapsedTime"].update(fswModeMgtState.aocsModeElapsedTime)

	# Manage events
	for event in allEvents:
		event.display()

	return fswModeMgtBusOut
	

def aocsGuidanceModeLogic(aocsMode):
	if (aocsMode == "SAFE"):
		aocsGuidMode = "GUIDMODE_RATE_DAMPING"
	elif (aocsMode == "NOM_PTNG"):
		aocsGuidMode = "GUIDMODE_ATT_INERT" 
	elif (aocsMode == "NOM_EQ"):
		aocsGuidMode = "GUIDMODE_ATT_NADIR"
	elif (aocsMode == "OCM"):
		aocsGuidMode = "GUIDMODE_ATT_INERT"
	elif (aocsMode == "MANUAL"):
		aocsGuidMode = "GUIDMODE_OFF"
        # TBW: GUIDMODE_OFF for now
	else:
		aocsGuidMode = "GUIDMODE_OFF"

	return aocsGuidMode

def aocsCtrModeLogic(aocsMode):
	if (aocsMode == "SAFE"):
		aocsCtrMode = "CTRLMODE_RATE_DAMP_CTRL"
		aocsCtrActMode = "THR"
	elif (aocsMode == "NOM_PTNG"):
		aocsCtrMode = "CTRLMODE_ATT_CTRL"
		aocsCtrActMode = "THR"
	elif (aocsMode == "NOM_EQ"):
		aocsCtrMode = "CTRLMODE_ATT_CTRL"
		aocsCtrActMode = "THR"
	elif (aocsMode == "OCM"):
		aocsCtrMode = "CTRLMODE_THRUST_CTRL"
		aocsCtrActMode = "THR"
	elif (aocsMode == "MANUAL"):
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