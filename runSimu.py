# -*- coding: utf-8 -*-
"""
Created on Fri Jul 19 14:44:02 2024

@author: Xela
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import ephem


import src.utils.simParam as simParam
import src.utils.constants as const
import src.utils.conversions as conversions
import src.utils.plots as plots

import src.data.initData as initData

import src.fsw.fswModel as fswModel
import src.fsw.fswGuidance as fswGuidance
import src.fsw.fswControl as fswControl
import src.fsw.fswEstimation as fswEstimation 

import src.attitudeDynamics as attitudeDynamics
import src.orbitDynamics as orbitDynamics

import src.models.scModel as scModel
import src.models.env.envModel as envModel
import src.models.act.actModels as actModels
import src.models.sen.senModels as senModels
import src.models.dyn.massModels as massModels

# Retrieve useful constants
pi  = np.pi
deg2rad = const.deg2rad
earthRadius = const.earthRadius
earthMu = const.earthMu


# --------------------------------------------------
# SETUP
# --------------------------------------------------
print("Setup...")
# Datetime
dateTimeStart = ephem.Date("2024/3/9 5:10:10")

# Simulation timestep and duration
Ts = 1 # [s]
Tend = 5*60 # [s]

# Simulation options
isGGTorqueEnabled = False
isCtrlTorqueEnabled = True
isPlot = True

# S/C attitude and angular rates
angRate_BI_B = np.array([1, 0.2, -0.3])*deg2rad

# S/C properties
# Dimensions for rectangle prism
massSc = 1000 # [kg]
lx = 2 # [m]
ly = 2 # [m]
lz = 4 # [m]
inertiaSc_B = massModels.getUniformRectanglePrismInertia(massSc, lx, ly, lz)

# Orbit
perAltitudeInit = 500 * 1e3 # [m]
ecc = 0.1
inc = 45 * deg2rad
raan = 90 * deg2rad
argPer = 90 * deg2rad
ta = 0 * deg2rad

# Attitude guidance
# GUIDMODE_OFF
# GUIDMODE_RATE_DAMPING
# GUIDMODE_ATT_NADIR
# GUIDMODE_ATT_INERT
attitudeGuidanceMode = "GUIDMODE_RATE_DAMPING"
GUIDMODE_ATT_INERT_eulerAngGuid_BI = np.array([5, 2, 1]) * deg2rad

# Attitude control
# CTRLMODE_ATT_CTRL
# CTRLMODE_OFF
# CTRLMODE_RATE_DAMP_CTRL
fswControlMode = "CTRLMODE_RATE_DAMP_CTRL"

# --------------------------------------------------
# INITIALIZATION
# --------------------------------------------------
print("Initialization...")
# Elapsed time
elapsedTime = 0

# [Simulation] Parameters
# ==================================
print("   Simulation parameters")
simParam = simParam.SimParam(Ts, Tend)
simParam.runOptions.isPlot = isPlot
simParam.simOptions.isCtrlTorqueEnabled = isCtrlTorqueEnabled
simParam.simOptions.isGGTorqueEnabled = isGGTorqueEnabled

# Data buses and signals
(modelsBus, fswBus) = initData.initializeBusesAndSignals(simParam)

print("   FSW / models")

# [Models] Ephemerides and time parameters
# ==================================
# Datetime
currDateTime = dateTimeStart
# Bodies
marsEphem = ephem.Mars()
marsEphem.compute(currDateTime, epoch = simParam.ephemEpoch)

# [Models] Spacecraft parameters
# ==================================
# Mass
scParam = scModel.Spacecraft()
scParam.massParam.inertiaSc_B = inertiaSc_B
scParam.massParam.inertiaScInv_B = np.linalg.inv(scParam.massParam.inertiaSc_B)
scParam.massParam.inertiaSc_G = scParam.massParam.inertiaSc_B
scParam.massParam.massSc = massSc
scParam.massParam.massCB = scParam.massParam.massSc
# Actuators
scParam.actParam = actModels.ActModelParam(scParam.massParam)
scParam.actParam.thrModelParam.updateThrSets(1, scParam.massParam);
scParam.actParam.thrModelParam.thrSets[0].isOn = True
# Sensors
scParam.senParam = senModels.SenModelParam()

# [FSW] Functions parameters
# ==================================
fswParam = fswModel.Fsw()

# [Guidance]
fswParam.guidParam.MODE = attitudeGuidanceMode
fswParam.guidParam.GUIDMODE_ATT_INERT_eulerAngGuid_BI = GUIDMODE_ATT_INERT_eulerAngGuid_BI

# [Control]
fswParam.ctrParam.MODE = fswControlMode

# [Models] Attitude and orbit states
# ==================================
# Orbit state
# Temporary definitions for orbit configuration => to be handled later with dedicated classes / functions
perRadiusInit = perAltitudeInit + earthRadius
sma = perRadiusInit / (1-ecc)
modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["sma"].update(sma)
modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["ecc"].update(ecc)
modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["inc"].update(inc)
modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["raan"].update(raan)
modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["argPer"].update(argPer)
modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["ta"].update(ta)

(pos_I, vel_I) = orbitDynamics.elementsToPosVelInertial(sma, ecc, inc, raan, argPer, ta, earthMu, modelsBus.subBuses["dynamics"].subBuses["orbitElem"])
orbit = orbitDynamics.Orbit(earthMu, pos_I, vel_I)
modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].update(pos_I)
modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["vel_I"].update(vel_I)

# Attitude state
myDynState = attitudeDynamics.DynamicsState()
myDynState.angRate_BI_B = angRate_BI_B
modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].update(angRate_BI_B)
modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].update(myDynState.qBI.toEuler())
modelsBus = attitudeDynamics.computeAngMom(scParam.massParam, modelsBus) 

# LVLH frame
lvlhFrame = envModel.LVLHframe()
lvlhFrame.update(orbit, myDynState)
modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_B"].update(lvlhFrame.zL_B)
modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_I"].update(lvlhFrame.zL_I)

# [Models] Spacecraft sensor state
# ==================================
modelsBus.subBuses["sensors"] = scParam.senParam.computeMeasurements(modelsBus.subBuses["sensors"], modelsBus)

# [FSW] Functions states
# ==================================
# [Estimation]
fswBus.subBuses["estimation"] = fswEstimation.attitudeEstimation(fswParam.estParam, fswBus, modelsBus)

# [Guidance]
fswBus.subBuses["guidance"] = fswGuidance.computeGuidance(fswParam.guidParam, lvlhFrame, modelsBus, fswBus)

# [Control]
fswBus.subBuses["control"] = fswControl.computeControl(fswParam.ctrParam, fswBus)

# [Models] Environment / actuators state
# ==================================
# Disturbance torque
modelsBus.subBuses["environment"] = envModel.computeExtTorque(simParam, scParam.massParam, orbit, lvlhFrame, modelsBus.subBuses["environment"])
 
# Spacecraft actuators torque
modelsBus.subBuses["actuators"] = scParam.actParam.computeActTorque(simParam.simOptions, fswBus.subBuses["control"], modelsBus.subBuses["actuators"])

# Total external torque
modelsBus = attitudeDynamics.computeTotTorque(modelsBus)

# --------------------------------------------------
# SIMULATION
# --------------------------------------------------
print("Simulation...")
simuStrTime = time.time()

for ii in range(1, simParam.nbPts):
    
    elapsedTime = elapsedTime + simParam.Ts
    
    # [Models]
    # ==================================
    # Ephemeris
    currDateTime = ephem.date(dateTimeStart + elapsedTime/(24*3600))
    marsEphem.compute(currDateTime, epoch = simParam.ephemEpoch)
    sunToMarsDir_I = conversions.latLonToCartesian(marsEphem.hlat/deg2rad, marsEphem.hlon/deg2rad, 1)
    
    # Environment torques
    modelsBus.subBuses["environment"] = envModel.computeExtTorque(simParam, scParam.massParam, orbit, lvlhFrame, modelsBus.subBuses["environment"])

    # Actuators
    modelsBus.subBuses["actuators"] = scParam.actParam.computeActTorque(simParam.simOptions, fswBus.subBuses["control"], modelsBus.subBuses["actuators"])

    # Total total external torque
    modelsBus = attitudeDynamics.computeTotTorque(modelsBus)
    
    # Propagate attitude dynamics
    myDynState.propagateState(scParam.massParam, simParam, modelsBus) 
    modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].update(myDynState.angRate_BI_B)
    modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].update(myDynState.qBI.toEuler())
    modelsBus = attitudeDynamics.computeAngMom(scParam.massParam, modelsBus) 

    # Propagate orbit dynamics
    orbit.propagate(simParam, earthMu)
    modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].update(orbit.pos_I)
    modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["vel_I"].update(orbit.vel_I)
    
    # Update LVLH frame
    lvlhFrame.update(orbit, myDynState)
    modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_B"].update(lvlhFrame.zL_B)
    modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_I"].update(lvlhFrame.zL_I)
    
    # Sensor
    modelsBus.subBuses["sensors"] = scParam.senParam.computeMeasurements(modelsBus.subBuses["sensors"], modelsBus)

    # [FSW]
    # ==================================
    # [Estimation] Compute estimated angular rates and / or attitude
    fswBus.subBuses["estimation"] = fswEstimation.attitudeEstimation(fswParam.estParam, fswBus, modelsBus)

    # [Guidance] Compute guidance angular rates and euler angles
    # Depends on the current guidance mode
    fswBus.subBuses["guidance"] = fswGuidance.computeGuidance(fswParam.guidParam, lvlhFrame, modelsBus, fswBus)

    # [Control] Compute control torque
    # Depends on the current control mode
    fswBus.subBuses["control"] = fswControl.computeControl(fswParam.ctrParam, fswBus)
    
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

plt.close("all")

plots.plotOrbit3d(modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].timeseries, const.earthRadius)

modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.rad2deg().plot()
modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.rad2deg().plot()
modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["torqueTot_B"].timeseries.plot()
modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].timeseries.plot()

modelsBus.subBuses["sensors"].signals["angRateMeas_BI_B"].timeseries.plot()
modelsBus.subBuses["actuators"].signals["torqueThr_B"].timeseries.plot()

modelsBus.subBuses["environment"].signals["torqueExt_B"].timeseries.plot()

fswBus.subBuses["guidance"].signals["angRateGuid_BI_B"].timeseries.rad2deg().plot()
fswBus.subBuses["control"].signals["torqueCtrl_B"].timeseries.plot()
