# -*- coding: utf-8 -*-
"""
Created on Fri Jul 19 14:44:02 2024

@author: Xela
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import ephem


import src.utils.simParam as simuParam
import src.utils.constants as const
import src.utils.conversions as conversions
import src.utils.plots as plots
import src.utils.paramTools as paramTools

import src.data.initData as initData
import src.data.spacecraftMassModelsDatabase as spacecraftMassModelsDatabase

import src.fsw.fswModel as fswModel
import src.fsw.fswModeMgt as fswModeMgt
import src.fsw.fswGuidance as fswGuidance
import src.fsw.fswEstimation as fswEstimation 
import src.fsw.fswControl as fswControl
import src.fsw.fswCommand as fswCommand

import src.attitudeDynamics as attitudeDynamics
import src.orbitDynamics as orbitDynamics
import src.attitudeKinematics as attitudeKinematics

import src.models.scModel as scModel
import src.models.spaceEnv.envModel as envModel
import src.models.spaceEnv.perfModel as perfModel
import src.models.act.actModels as actModels
import src.models.sen.senModels as senModels

# Retrieve useful constants
pi  = np.pi
deg2rad = const.deg2rad
earthRadius = const.earthRadius
earthMu = const.earthMu


# --------------------------------------------------
# SETUP
# --------------------------------------------------
print("Setup...")
# Simulation timestep and duration
# Ts = 1 # [s] 
# Tend = 2*90*60 # [s]

# Simulation parameters
simParamPatch = {}
simParamPatch["dateTimeStart"] = ephem.Date("2024/3/9 5:10:10")
simParamPatch["Ts"] = 1 # [s]
simParamPatch["Tend"] = 2*90*60 # [s]
runOptionsPatch = {}
runOptionsPatch["isPlot"] = True
simOptionsPatch = {}
simOptionsPatch["isGGTorqueEnabled"] = False
simOptionsPatch["isCtrlTorqueEnabled"] = True
simOptionsPatch["swInitAttitudeFrame"] = "I"
simOptionsPatch["massModelName"] = "SC_100kg_rect"

# S/C attitude and angular rates
angRate_BI_B = np.array([0, 0, 0])
qBI = attitudeKinematics.Quaternion()

# Orbit
orbitInitParam = orbitDynamics.OrbitInitParam()
perAltitudeInit = 500 * 1e3 # [m]
ecc = 0.01
inc = 0.1 * deg2rad
raan = 10 * deg2rad
argPer = 90 * deg2rad
ta = 0 * deg2rad

# Attitude
qInitVec = attitudeKinematics.trans_EulerAngToQuat(np.array([20, -40, 0]) * deg2rad).toVec()
angRateInit_B = np.array([0.1, 0.5, 0])*deg2rad

# Mode management
modeMgtParamPatch = {}
modeMgtParamPatch["aocsOffModeMinDur"] = 50*60
modeMgtParamPatch["aocsSafeModeAngRateThdDur"] = 50*60
modeMgtParamPatch["aocsSafeModeAngRateThd"] = 0.02*deg2rad
modeMgtParamPatch["isAutoSafeToNomPtngModeAllwd"] = True
modeMgtParamPatch["isAutoNomPtngToNomEqAllwd"] = True

# Control
attCtrNatFreq =  0.0010919*10 # [rad/s]
attCtrDamping = 0.6 # [-]
rateCtrTimeResp2pc = 10*60 # [s]
ctrParamPatch = {}
ctrParamPatch["swInertiaTrqCompensation"] = False

# Initial AOCS mode
# OFF (Off)
# SAFE (Safe)
# NOM_PTNG (Nominal - pointing)
# NOM_EQ (Nominal - equilibrium)
# NOM_SLEW (Nominal - slew) => To be added
# OCM (Orbit Control Mode)
aocsMode = "OFF"

# Guidance
guidParamPatch = {}
guidParamPatch["GUIDMODE_ATT_INERT_eulerAngGuid_RI"] = np.array([0, 10, 0]) * deg2rad

# --------------------------------------------------
# INITIALIZATION
# --------------------------------------------------
def initializeSimulation():
    
    print("Initialization...")    
    # [Simulation] Parameters
    # ==================================
    print("   Simulation parameters")
    simParam = simuParam.SimParam()
    simParam = paramTools.patchAttributes(simParam, simParamPatch)
    simParam.updateTimeVec()
    simParam.runOptions = paramTools.patchAttributes(simParam.runOptions, runOptionsPatch)
    simParam.simOptions = paramTools.patchAttributes(simParam.simOptions, simOptionsPatch)
    
    # Data buses and signals
    (modelsBus, fswBus, simBus) = initData.initializeBusesAndSignals(simParam)
    
    print("   FSW / models")
    
    # [Models] Spacecraft parameters
    # ==================================
    print("      [Models] Spacecraft parameters")
    # Mass
    scParam = scModel.Spacecraft(massModelName = simParam.simOptions.massModelName)
    
    # Actuators # TBW
    scParam.actParam = actModels.ActModelParam(scParam.massParam)
    scParam.actParam.thrModelParam.initThr(1, scParam.massParam);
    scParam.actParam.thrModelParam.thrSets[0].isOn = True # TBW
    # Sensors
    scParam.senParam = senModels.SenModelParam()
    
    # [Models] Orbit state
    # ==================================
    print("      [Models] Orbit state")
    (modelsBus, orbitInitObj) = orbitDynamics.setInitialOrbit(modelsBus, orbitInitParam)
    
    # LVLH frame => initialized here because it is needed for the attitude initalization
    # the initial attitude can be defined wrt the LVLH frame
    modelsBus = envModel.computeLvlhFrame(modelsBus)
    
    # [Models] Attitude state
    # ==================================
    print("      [Models] Attitude state")
    modelsBusOut = attitudeDynamics.setInitialAttitude(modelsBus, simParam, qInitVec = qInitVec, angRateInit_B = angRateInit_B)
            
    # Performance (additional models signals)
    modelsBus = perfModel.computeBodyNadirAttitude(modelsBus)
    
    # [Models] Spacecraft sensors states
    # ==================================
    print("      [Models] Spacecraft sensors states")
    modelsBus.subBuses["sensors"] = scParam.senParam.computeMeasurements(modelsBus.subBuses["sensors"], modelsBus)
    
    # [FSW] Functions parameters
    # ==================================
    print("      [FSW] Functions parameters")
    # Initialize
    fswParam = fswModel.Fsw(scParam)
    
    # [Guidance] Patch simulation-specific parameters
    #fswParam.guidParam.GUIDMODE_ATT_INERT_eulerAngGuid_RI = GUIDMODE_ATT_INERT_eulerAngGuid_RI
    fswParam.guidParam = paramTools.patchAttributes(fswParam.guidParam, guidParamPatch)
    
    # [Control] Patch simulation-specific parameters
    fswParam.ctrParam.inertiaSc_B = scParam.massParam.inertiaSc_B
    fswParam.ctrParam = paramTools.patchAttributes(fswParam.ctrParam, ctrParamPatch)
    # Tuning => TBW
    fswParam.ctrParam.getAttCtrlTuningFrom_dampingAndNaturalFrequency(attCtrDamping, attCtrNatFreq)
    fswParam.ctrParam.getRateCtrlTuningFrom_timeResp(rateCtrTimeResp2pc)
    
    # [Command] Patch with context data (from models parameters)
    # FSW has full knowledge of actuator model => TBW to be descoped
    fswParam.cmdParam.thrCmdParam = scParam.actParam.thrModelParam
    fswParam.cmdParam.rwCmdParam = scParam.actParam.rwModelParam
    
    # [Mode management] Patch simulation-specific parameters
    fswModeMgtState = fswModeMgt.FswModeMgtState(aocsMode) # TBW
    fswParam.modeMgtParam = paramTools.patchAttributes(fswParam.modeMgtParam, modeMgtParamPatch)
    
    # [FSW] Functions states
    # ==================================
    print("      [FSW] Functions states")
    
    # [Mode management]
    fswBus.subBuses["modeMgt"] = fswModeMgt.computeModeMgt(simParam, fswParam, fswModeMgtState, fswBus)
    
    # [Estimation]
    fswBus.subBuses["estimation"] = fswEstimation.attitudeEstimation(fswParam.estParam, fswBus, modelsBus)
    
    # [Guidance]
    fswParam.guidParam.orbitRate = orbitInitObj.orbitContext.orbitPulse
    fswBus.subBuses["guidance"] = fswGuidance.computeGuidance(fswParam.guidParam, modelsBus.subBuses["environment"], fswBus)
    
    # [Control]
    fswBus.subBuses["control"] = fswControl.computeControl(fswParam.ctrParam, fswBus)
    
    # [Command]
    fswBus.subBuses["command"] = fswCommand.computeCommand(fswParam.cmdParam, fswBus)
    
    # [Models] Environment / actuators states
    # ==================================
    print("      [Models] Environment / actuators states")
    # Ephemerides
    modelsBus.subBuses["environment"] = envModel.computeEphemerides(simBus, modelsBus, simParam)

    # Disturbance torque
    modelsBus = envModel.computeExtTorque(simParam, scParam.massParam, modelsBus)
     
    # Spacecraft actuators torque
    modelsBus.subBuses["actuators"] = scParam.actParam.computeActTorque(simParam.simOptions, fswBus.subBuses["command"], modelsBus.subBuses["actuators"])
    
    # Total external torque
    modelsBus = attitudeDynamics.computeTotTorque(modelsBus)
    
    # To be encapsulated:
    # fswModeMgtState
        
    return (simParam, fswParam, scParam, fswModeMgtState, modelsBus, fswBus, simBus)

(simParam, fswParam, scParam, fswModeMgtState, modelsBus, fswBus, simBus) = initializeSimulation()


# --------------------------------------------------
# SIMULATION
# --------------------------------------------------
print("Simulation...")
simuStrTime = time.time()

for ii in range(1, simParam.nbPts):
    
    # [Simulation]
    # ==================================
    elapsedTimePrev = simBus.signals["elapsedTime"].value
    simBus.signals["elapsedTime"].update(elapsedTimePrev + simParam.Ts)

    # [Models]
    # ==================================    
    # Environment
    modelsBus.subBuses["environment"] = envModel.computeEphemerides(simBus, modelsBus, simParam)
    modelsBus = envModel.computeExtTorque(simParam, scParam.massParam, modelsBus)

    # Actuators
    modelsBus.subBuses["actuators"] = scParam.actParam.computeActTorque(simParam.simOptions, fswBus.subBuses["command"], modelsBus.subBuses["actuators"])

    # Total total external torque
    modelsBus = attitudeDynamics.computeTotTorque(modelsBus)
    
    # Propagate attitude dynamics
    modelsBus = attitudeDynamics.propagateAttitude(scParam.massParam, simParam, modelsBus)

    # Compute spacecraft angular momentum
    modelsBus = attitudeDynamics.computeAngMom(scParam.massParam, modelsBus) 

    # Propagate orbit dynamics
    modelsBus = orbitDynamics.propagateOrbit(simParam, modelsBus)

    # Sensor
    modelsBus.subBuses["sensors"] = scParam.senParam.computeMeasurements(modelsBus.subBuses["sensors"], modelsBus)

    # LVLH frame
    modelsBus = envModel.computeLvlhFrame(modelsBus)

    # Performance (additional models signals)
    modelsBus = perfModel.computeBodyNadirAttitude(modelsBus)

    # [FSW]
    # ==================================
    # [Mode management]
    fswBus.subBuses["modeMgt"] = fswModeMgt.computeModeMgt(simParam, fswParam, fswModeMgtState, fswBus)

    # [Estimation] Compute estimated angular rates and / or attitude
    fswBus.subBuses["estimation"] = fswEstimation.attitudeEstimation(fswParam.estParam, fswBus, modelsBus)

    # [Guidance] Compute guidance angular rates and euler angles
    # Depends on the current guidance mode
    fswBus.subBuses["guidance"] = fswGuidance.computeGuidance(fswParam.guidParam, modelsBus.subBuses["environment"], fswBus)

    # [Control] Compute control torque
    # Depends on the current control mode
    fswBus.subBuses["control"] = fswControl.computeControl(fswParam.ctrParam, fswBus)

    # [Command] Compute actuator commands
    fswBus.subBuses["command"] = fswCommand.computeCommand(fswParam.cmdParam, fswBus)

    
simuEndTime = time.time()
simuDuration = simuEndTime - simuStrTime
print("   Duration: " + str(simuDuration))
    
# --------------------------------------------------
# DEBUG
# --------------------------------------------------


# --------------------------------------------------
# POST-PROCESSING
# --------------------------------------------------
print("Post-processing...")

# --------------------------------------------------
# PLOTS
# --------------------------------------------------
print("Plots...")

#plt.show(block = False)
plt.pause(3)
plt.close("all")


#plots.plotOrbit3d(modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].timeseries, const.earthRadius)


modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.rad2deg().plot()
fswBus.subBuses["guidance"].signals["angRate_RI_R"].timeseries.rad2deg().plot()

modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.rad2deg().plot()
fswBus.subBuses["guidance"].signals["eulerAng_RI"].timeseries.rad2deg().plot()
modelsBus.subBuses["environment"].signals["eulerAng_LI"].timeseries.rad2deg().plot()
modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["torqueTot_B"].timeseries.addNorm().plot()
#modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].timeseries.addNorm().plot()

fswBus.subBuses["control"].signals["eulerAngEst_BR"].timeseries.rad2deg().plot()
modelsBus.subBuses["performance"].signals["eulerAng_BL"].timeseries.rad2deg().plot()
modelsBus.subBuses["performance"].signals["angRate_BL_B"].timeseries.rad2deg().plot()

# modelsBus.subBuses["sensors"].signals["angRateMeas_BI_B"].timeseries.addNorm().rad2deg().plot()
# modelsBus.subBuses["actuators"].signals["torqueThr_B"].timeseries.addNorm().plot()

#modelsBus.subBuses["environment"].signals["torqueExt_B"].timeseries.addNorm().plot()

# fswBus.subBuses["guidance"].signals["angRate_RI_R"].timeseries.rad2deg().plot()
# fswBus.subBuses["guidance"].signals["eulerAng_RI"].timeseries.rad2deg().plot()

fswBus.subBuses["control"].signals["qEstBR_sca"].timeseries.plot()
fswBus.subBuses["control"].signals["qEstBR_vec"].timeseries.plot()
fswBus.subBuses["control"].signals["rotAngEst_BR"].timeseries.rad2deg().plot()
fswBus.subBuses["control"].signals["angRateEst_BR_B"].timeseries.rad2deg().plot()
#fswBus.subBuses["control"].signals["forceCtrl_B"].timeseries.addNorm().plot()
#fswBus.subBuses["control"].signals["torqueCtrl_B"].timeseries.addNorm().plot()
fswBus.subBuses["control"].signals["torqueCtrlThr_B"].timeseries.plot()
#fswBus.subBuses["control"].signals["torqueCtrlRw_B"].timeseries.plot()

#fswBus.subBuses["command"].signals["torqueCmdRw_B"].timeseries.addNorm().plot()
#fswBus.subBuses["command"].signals["torqueCmdThr_B"].timeseries.addNorm().plot()

#fswBus.subBuses["modeMgt"].signals["aocsModeElapsedTime"].timeseries.plot()

modelsBus.subBuses["environment"].signals["qLI_sca"].timeseries.plot()
modelsBus.subBuses["environment"].signals["qLI_vec"].timeseries.plot()

modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["ta"].timeseries.rad2deg().plot()
modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["argPer"].timeseries.rad2deg().plot()


