# -*- coding: utf-8 -*-
"""
Created on Fri Jul 19 14:44:02 2024

@author: Xela
"""

import yaml # [WIP]
from deepmerge import always_merger # [WIP]
from pathlib import Path # [WIP]

import src.utils.simulation as sim
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

import src.models.models as mod
import src.models.dyn.attitudeDynamics as attDyn
import src.models.dyn.orbitDynamics as orbDyn

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
    # Get patches
    patches = scenarioPatchFcnHdl()

    # Load external data
    allParam = load_data(DEFAULTDATA_PATH)


    print("Creating simulation object...")    
    simulation = sim.Simulation()

    print("Setting parameters...")  
    # [Simulation] Parameters
    # ==================================
    print("   Simulation parameters")
    simulation.param = paramTools.patchAttributes(simulation.param, patches["simParamPatch"])
    simulation.param.updateTimeVec()
    simulation.param.runOptions = paramTools.patchAttributes(simulation.param.runOptions, patches["runOptionsPatch"])
    simulation.param.simOptions = paramTools.patchAttributes(simulation.param.simOptions, patches["simOptionsPatch"])
    # Create simulation object
    simulation = sim.Simulation()

    # Data buses and signals
    (modelsBus, fswBus, simBus) = initData.initializeBusesAndSignals(simulation.param)
    
    # Interfaces parameters
    # ==================================
    print("   Interfaces")
    interfaceInputsParam = interfaceInputs.Param()
    interfaceInputsParam.keyboardInputsParam = paramTools.patchAttributes(interfaceInputsParam.keyboardInputsParam, patches["ifKeyboardParamPatch"])
    joystickSerialPort = interfaceInputsParam.joystickParam.initializeSerialPort()

    print("   FSW / models")
    # [Models] Models
    # ==================================
    print("      [Models] Models")
    models = mod.Models(simulation.param.simOptions.massModelName, allParam)
    models.dynamics.orbit.config = paramTools.patchAttributes(models.dynamics.orbit.config, patches["orbitInitParamPatch"])
    models.dynamics.orbit.set()

    # Performance
    # TBW: Implement performance model as class Performance
    # modelsBus = perfModel.computeBodyNadirAttitude(modelsBus)
    
    # [FSW] Functions parameters
    # ==================================
    print("      [FSW] Functions parameters")
    # Initialize
    fsw = fswModel.Fsw()
    
    # [Guidance] Patch simulation-specific parameters
    fsw.guidance.param = paramTools.patchAttributes(fsw.guidance.param, patches["guidParamPatch"])
    
    # [Control] Patch simulation-specific parameters
    fsw.control.param.inertiaSc_B = models.spacecraft.massModel.inertiaSc_B
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
    fsw.modeMgt.state = fswModeMgt.State(fsw.modeMgt.param.aocsModeInit)
    
    # [FSW] Functions states
    # ==================================
    receivedTcList = []
    fsw.run(simulation, models.interface, receivedTcList)
    
    # [Models]
    # ==================================
    # print("      [Models] Spacecraft and environment states")
    # modelsBus = models.run(simulation.param, simBus, fswBus, modelsBus)
        
    # [Displays]
    # ==================================
    print("      [Displays]")
    displays = 0
    # displays = display.Displays()
    #displays.initialize(simulation.param, modelsBus, fswBus, simBus, fswModeMgtState)
        
    return (simulation, interfaceInputsParam, fsw, models, modelsBus, fswBus, simBus, joystickSerialPort, displays)
