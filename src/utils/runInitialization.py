# -*- coding: utf-8 -*-
"""
Created on Fri Jul 19 14:44:02 2024

@author: Xela
"""

import yaml # [WIP]
from deepmerge import always_merger # [WIP]
from pathlib import Path # [WIP]

import src.utils.simParam as simuParam
import src.utils.paramTools as paramTools

import src.data.initData as initData

import src.fsw.fswModel as fswModel
import src.fsw.fswModeMgt as fswModeMgt
import src.fsw.fswGuidance as fswGuidance
import src.fsw.fswEstimation as fswEstimation 
import src.fsw.fswControl as fswControl
import src.fsw.fswCommand as fswCommand

import src.interfaces.interfaceInputs as interfaceInputs
import src.interfaces.display as display
import src.interfaces.eventsAndTmTc as eventsAndTmTc

import src.models.dyn.attitudeDynamics as attitudeDynamics
import src.models.dyn.orbitDynamics as orbitDynamics

import src.models.scModel as scModel
import src.models.spaceEnv.envModel as envModel
import src.models.spaceEnv.perfModel as perfModel
import src.models.act.actModels as actModels
import src.models.sen.senModels as senModels
import src.models.act.thrModels as thrModels # [WIP]


PROJECTROOT_PATH = Path(__file__).resolve().parents[2]
DEFAULTDATA_PATH = PROJECTROOT_PATH / "data"

# [WIP]
def load_data(data_directory):    
    # Initialize doctionnary
    defaultData = {}
    # Parse .yaml files recursively in the data directory
    for file in Path(data_directory).rglob("*.yaml"):
        data = yaml.safe_load(file.read_text())
        defaultData = always_merger.merge(defaultData, data)
    return defaultData


def runInitialization(scenarioPatchFcnHdl):
    print("Retrieving scenario-specific patches...")    
    # Applying the patches
    # ==================================
    patches = scenarioPatchFcnHdl()

    # [WIP] Load .yaml data
    allParam = load_data(DEFAULTDATA_PATH)

    print("Setting parameters...")  
    # [Simulation] Parameters
    # ==================================
    print("   Simulation parameters")
    simParam = simuParam.SimParam()
    simParam = paramTools.patchAttributes(simParam, patches["simParamPatch"])
    simParam.updateTimeVec()
    simParam.runOptions = paramTools.patchAttributes(simParam.runOptions, patches["runOptionsPatch"])
    simParam.simOptions = paramTools.patchAttributes(simParam.simOptions, patches["simOptionsPatch"])
    
    # Data buses and signals
    (modelsBus, fswBus, simBus) = initData.initializeBusesAndSignals(simParam)
    
    # Interfaces parameters
    # ==================================
    print("   Interfaces")
    interfaceInputsParam = interfaceInputs.Param()
    interfaceInputsParam.keyboardInputsParam = paramTools.patchAttributes(interfaceInputsParam.keyboardInputsParam, patches["ifKeyboardParamPatch"])
    joystickSerialPort = interfaceInputsParam.joystickParam.initializeSerialPort()

    print("   FSW / models")
    
    # [Models] Spacecraft parameters
    # ==================================
    print("      [Models] Spacecraft parameters")
    # Mass
    scParam = scModel.Spacecraft(simParam.simOptions.massModelName, allParam)
    
    # [Models] Orbit state
    # ==================================
    print("      [Models] Orbit state")
    orbitInitParam = orbitDynamics.OrbitInitParam()
    orbitInitParam = paramTools.patchAttributes(orbitInitParam, patches["orbitInitParamPatch"])
    (modelsBus, orbitInitObj) = orbitDynamics.setInitialOrbit(modelsBus, orbitInitParam)
    
    # LVLH frame => initialized here because it is needed for the attitude initalization
    # the initial attitude can be defined wrt the LVLH frame
    modelsBus = envModel.computeLvlhFrame(modelsBus)
    
    # [Models] Attitude state
    # ==================================
    print("      [Models] Attitude state")
    initDynState = attitudeDynamics.DynamicsState()
    initDynState = paramTools.patchAttributes(initDynState, patches["initDynStatePatch"])
    modelsBus = attitudeDynamics.setInitialAttitude(modelsBus, simParam, initDynState)
            
    # Performance (additional models signals)
    modelsBus = perfModel.computeBodyNadirAttitude(modelsBus)
    
    # [Models] Spacecraft sensors states
    # ==================================
    print("      [Models] Spacecraft sensors states")
    modelsBus.subBuses["sensors"] = scParam.sen.computeMeasurements(modelsBus.subBuses["sensors"], modelsBus)
    
    # [FSW] Functions parameters
    # ==================================
    print("      [FSW] Functions parameters")
    # Initialize
    fsw = fswModel.Fsw()
    
    # [Guidance] Patch simulation-specific parameters
    fsw.guidance.param = paramTools.patchAttributes(fsw.guidance.param, patches["guidParamPatch"])
    
    # [Control] Patch simulation-specific parameters
    fsw.control.param.inertiaSc_B = scParam.massModel.inertiaSc_B
    fsw.control.param = paramTools.patchAttributes(fsw.control.param, patches["ctrParamPatch"])
    # Tuning => TBW
    attCtrNatFreq =  0.0010919*10 # [rad/s]
    attCtrDamping = 0.6 # [-]
    rateCtrTimeResp2pc = 10*60 # [s]
    fsw.control.param.getAttCtrlTuningFrom_dampingAndNaturalFrequency(attCtrDamping, attCtrNatFreq)
    fsw.control.param.getRateCtrlTuningFrom_timeResp(rateCtrTimeResp2pc)
    
    # FSW has full knowledge of actuator model => TBW to be descoped
    # TBW
    
    # [Mode management] Patch simulation-specific parameters    
    fsw.modeMgt.param = paramTools.patchAttributes(fsw.modeMgt.param, patches["modeMgtParamPatch"])
    fsw.modeMgt.state = fswModeMgt.FswModeMgtState(fsw.modeMgt.param.aocsModeInit)
    
    # [FSW] Functions states
    # ==================================
    print("      [FSW] Functions states")
    
    # [Mode management]
    (fswBus.subBuses["modeMgt"], modeMgtEvtList) = fsw.modeMgt.run(simParam, fswBus, simBus, [])
    
    # [Interfaces]
    fswBus.subBuses["interfaces"] = interfaceInputs.getInterfaceInputs(fswBus, simBus, joystickSerialPort, interfaceInputsParam)

    # [Estimation]
    fswBus.subBuses["estimation"] = fsw.estimation.run(fswBus, modelsBus)
    
    # [Guidance]
    fsw.guidance.param.orbitRate = orbitInitObj.orbitContext.orbitPulse
    fswBus.subBuses["guidance"] = fsw.guidance.run(fswBus)
    
    # [Control]
    fswBus.subBuses["control"] = fsw.control.run(fswBus)
       
    # [Command]
    fswBus.subBuses["command"] = fsw.command.run(fswBus)
    
    # [Models] Environment / actuators states
    # ==================================
    print("      [Models] Environment / actuators states")
    # Ephemerides
    modelsBus.subBuses["environment"] = envModel.computeEphemerides(simBus, modelsBus, simParam)

    # Disturbance torque
    modelsBus = envModel.computeExtTorque(simParam, scParam.massModel, modelsBus)
     
    # Spacecraft actuators torque
    modelsBus.subBuses["actuators"] = scParam.act.computeActTorque(simParam.simOptions, fswBus.subBuses["command"], modelsBus.subBuses["actuators"])
    
    # Total external torque
    modelsBus = attitudeDynamics.computeTotTorque(modelsBus)
        
    # [Displays]
    # ==================================
    print("      [Displays]")
    displays = 0
    # displays = display.Displays()
    #displays.initialize(simParam, modelsBus, fswBus, simBus, fswModeMgtState)
        
    return (simParam, interfaceInputsParam, fsw, scParam, modelsBus, fswBus, simBus, joystickSerialPort, displays)
