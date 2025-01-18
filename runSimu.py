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

import src.fsw.fswGuidance as fswGuidance
import src.fsw.fswControl as fswControl
import src.fsw.fswEstimation as fswEstimation 

import src.attitudeDynamics as attitudeDynamics
import src.orbitDynamics as orbitDynamics


import src.models.env.envModel as envModel
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
# Datetime
dateTimeStart = ephem.Date("2024/3/9 5:10:10")

# Simulation timestep and duration
Ts = 1 # [s]
Tend = 90*60 # [s]

# Simulation options
isGGTorqueEnabled = True
isCtrlTorqueEnabled = True
isPlot = True

# S/C attitude and angular rates
angRate_BI_B = np.array([1, 0.1, 0.1])*deg2rad

# S/C mass properties
inertiaSc_B = np.diag([100, 120, 20])

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
attitudeGuidanceMode = "GUIDMODE_ATT_INERT"
GUIDMODE_ATT_INERT_eulerAngGuid_BI = np.array([5, 2, 1]) * deg2rad

# Attitude control
# CTRLMODE_ATT_CTRL
# CTRLMODE_OFF
# CTRLMODE_RATE_DAMP_CTRL
fswControlMode = "CTRLMODE_ATT_CTRL"

# --------------------------------------------------
# INITIALIZATION
# --------------------------------------------------
print("Initialization...")
# Elapsed time
elapsedTime = 0

# [Simulation] Parameters
print("   Simulation parameters")
simParam = simParam.SimParam(Ts, Tend, isGGTorqueEnabled, isCtrlTorqueEnabled, isPlot)

# Data buses and signals
(modelsBus, fswBus) = initData.initializeBusesAndSignals(simParam)

print("   FSW / models")

# [Ephemeris] Datetime
currDateTime = dateTimeStart
# [Ephemeris] Bodies
marsEphem = ephem.Mars()
marsEphem.compute(currDateTime, epoch = simParam.ephemEpoch)

# [Models] Orbit
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

# [Models] Dynamics properties
attDynParam = attitudeDynamics.AttDynParam()
attDynParam.inertiaSc_B = inertiaSc_B
attDynParam.inertiaSc_G = attDynParam.inertiaSc_B

# [Models] Attitude
myDynState = attitudeDynamics.DynamicsState()
myDynState.angRate_BI_B = angRate_BI_B
modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].update(angRate_BI_B)
modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].update(myDynState.qBI.toEuler())

# [Models] LVLH frame
lvlhFrame = envModel.LVLHframe()
lvlhFrame.update(orbit, myDynState)
modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_B"].update(lvlhFrame.zL_B)
modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_I"].update(lvlhFrame.zL_I)

# [Models] Sensors
# Parameters
senModelParam = senModels.SenModelParam()
# Initial value
modelsBus.subBuses["sensors"] = senModelParam.computeMeasurements(modelsBus.subBuses["sensors"], modelsBus)

# [FSW] Estimation
# Parameters
fswEstimationParam = fswEstimation.FswEstimationParam()
# Initial value
fswBus.subBuses["estimation"] = fswEstimation.attitudeEstimation(fswEstimationParam, fswBus, modelsBus)
 
# [FSW] Guidance
# Parameters
fswGuidanceParam = fswGuidance.FswGuidanceParam()
fswGuidanceParam.MODE = attitudeGuidanceMode
fswGuidanceParam.GUIDMODE_ATT_INERT_eulerAngGuid_BI = GUIDMODE_ATT_INERT_eulerAngGuid_BI
# Initial value
fswBus.subBuses["guidance"] = fswGuidance.computeGuidance(fswGuidanceParam, lvlhFrame, modelsBus, fswBus)

# [FSW] Control
# Parameters
fswControlParam = fswControl.FswControlParam()
fswControlParam.MODE = fswControlMode 
# Initial value
fswBus.subBuses["control"] = fswControl.computeControl(fswControlParam, fswBus)

# [Models] Environment
modelsBus.subBuses["environment"] = envModel.computeExtTorque(simParam, attDynParam, orbit, lvlhFrame, modelsBus.subBuses["environment"])

# [Models] Actuators
# Parameters
actModelParam = actModels.ActModelParam(attDynParam)
actModelParam.thrModelParam.updateThrSets(1, attDynParam);
actModelParam.thrModelParam.thrSets[0].isOn = True

# Initial value
modelsBus.subBuses["actuators"] = actModelParam.computeActTorque(simParam.simOptions, fswBus.subBuses["control"], modelsBus.subBuses["actuators"])

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
    modelsBus.subBuses["environment"] = envModel.computeExtTorque(simParam, attDynParam, orbit, lvlhFrame, modelsBus.subBuses["environment"])

    # Actuators
    modelsBus.subBuses["actuators"] = actModelParam.computeActTorque(simParam.simOptions, fswBus.subBuses["control"], modelsBus.subBuses["actuators"])

    # Total total external torque
    modelsBus = attitudeDynamics.computeTotTorque(modelsBus)
    
    # Propagate attitude dynamics
    myDynState.propagateState(attDynParam, simParam, modelsBus) 
    modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].update(myDynState.angRate_BI_B)
    modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].update(myDynState.qBI.toEuler())

    # Propagate orbit dynamics
    orbit.propagate(simParam, earthMu)
    modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].update(orbit.pos_I)
    modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["vel_I"].update(orbit.vel_I)
    
    # Update LVLH frame
    lvlhFrame.update(orbit, myDynState)
    modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_B"].update(lvlhFrame.zL_B)
    modelsBus.subBuses["dynamics"].subBuses["frames"].signals["zL_I"].update(lvlhFrame.zL_I)
    
    # Sensor
    modelsBus.subBuses["sensors"] = senModelParam.computeMeasurements(modelsBus.subBuses["sensors"], modelsBus)

    # [FSW]
    # ==================================
    # [Estimation] Compute estimated angular rates and / or attitude
    fswBus.subBuses["estimation"] = fswEstimation.attitudeEstimation(fswEstimationParam, fswBus, modelsBus)

    # [Guidance] Compute guidance angular rates and euler angles
    # Depends on the current guidance mode
    fswBus.subBuses["guidance"] = fswGuidance.computeGuidance(fswGuidanceParam, lvlhFrame, modelsBus, fswBus)

    # [Control] Compute control torque
    # Depends on the current control mode
    fswBus.subBuses["control"] = fswControl.computeControl(fswControlParam, fswBus)
    
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

modelsBus.subBuses["environment"].signals["torqueExt_B"].timeseries.plot()

fswBus.subBuses["guidance"].signals["angRateGuid_BI_B"].timeseries.rad2deg().plot()
fswBus.subBuses["control"].signals["torqueCtrl_B"].timeseries.plot()

