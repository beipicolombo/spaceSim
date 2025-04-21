# -*- coding: utf-8 -*-
"""
Created on Mon Apr 21 01:16:29 2025

@author: Xela
"""

import time

import src.fsw.fswModeMgt as fswModeMgt
import src.fsw.fswGuidance as fswGuidance
import src.fsw.fswEstimation as fswEstimation 
import src.fsw.fswControl as fswControl
import src.fsw.fswCommand as fswCommand

import src.interfaces.interfaceInputs as interfaceInputs

import src.attitudeDynamics as attitudeDynamics
import src.orbitDynamics as orbitDynamics
import src.attitudeKinematics as attitudeKinematics

import src.models.spaceEnv.envModel as envModel
import src.models.spaceEnv.perfModel as perfModel


def runLoop(simParam, interfaceInputsParam, fswParam, scParam, fswModeMgtState, modelsBus, fswBus, simBus, joystickSerialPort, displays):
    
    simuStrTime = time.time()

    for ii in range(1, simParam.nbPts):    
        # time.sleep(simParam.Ts)
        
        # [Simulation]
        # ==================================
        elapsedTimePrev = simBus.signals["elapsedTime"].value
        simBus.signals["elapsedTime"].update(elapsedTimePrev + simParam.Ts)

        # [Models]
        # ==================================    
        # Environment
        # 1. Get the previous quaternion for nadir frame => to be used for animation
        qPrevLI_sca = modelsBus.subBuses["environment"].signals["qLI_sca"].value
        qPrevLI_vec = modelsBus.subBuses["environment"].signals["qLI_vec"].value
        qPrevLI = attitudeKinematics.Quaternion(qPrevLI_sca, qPrevLI_vec)
        # 2. Update environment
        modelsBus.subBuses["environment"] = envModel.computeEphemerides(simBus, modelsBus, simParam)
        modelsBus = envModel.computeExtTorque(simParam, scParam.massParam, modelsBus)

        # Actuators
        modelsBus.subBuses["actuators"] = scParam.actParam.computeActTorque(simParam.simOptions, fswBus.subBuses["command"], modelsBus.subBuses["actuators"])

        # Total total external torque
        modelsBus = attitudeDynamics.computeTotTorque(modelsBus)
        
        # Propagate attitude dynamics
        # 1. Get the previous quaternion => to be used for animation
        qPrevBI_sca = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].value
        qPrevBI_vec = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].value
        qPrevBI = attitudeKinematics.Quaternion(qPrevBI_sca, qPrevBI_vec)
        # 2. Propagate
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

        # [Interfaces]
        fswBus.subBuses["interfaces"] = interfaceInputs.getInterfaceInputs(fswBus, joystickSerialPort, interfaceInputsParam)

        # [Estimation] Compute estimated angular rates and / or attitude
        fswBus.subBuses["estimation"] = fswEstimation.attitudeEstimation(fswParam.estParam, fswBus, modelsBus)

        # [Guidance] Compute guidance angular rates and euler angles
        # Depends on the current guidance mode
        fswBus.subBuses["guidance"] = fswGuidance.computeGuidance(fswParam.guidParam, modelsBus.subBuses["environment"], fswBus)

        # [Control] Compute control torque
        fswBus.subBuses["control"] = fswControl.computeControl(fswParam.ctrParam, fswBus)


        # [Command] Compute actuator commands
        fswBus.subBuses["command"] = fswCommand.computeCommand(fswParam.cmdParam, fswBus)

        # [Graphics]
        # ==================================    
        displays.update(simParam, modelsBus, fswBus, fswModeMgtState, qPrevLI, qPrevBI)

    simuEndTime = time.time()
    simuDuration = simuEndTime - simuStrTime
    print("   Duration: " + str(simuDuration))

    joystickSerialPort.close()
    
    return (modelsBus, fswBus, simBus)