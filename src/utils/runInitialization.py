# -*- coding: utf-8 -*-
"""
Created on Fri Jul 19 14:44:02 2024

@author: Xela
"""

import yaml # [WIP]
from deepmerge import always_merger # [WIP]
from pathlib import Path # [WIP]
import time

import src.utils.paramTools as paramTools
import src.utils.simulation as sim

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
    # Get start time
    simuStrTime = time.time()

    print("Retrieving scenario-specific patches...")    
    # Get patches
    patches = scenarioPatchFcnHdl()

    # Load external data
    allParam = load_data(DEFAULTDATA_PATH)

    # Initialize objects
    print("================")
    print("Initializing objects...")
    print("   simulation")
    simulation = sim.Simulation()

    print("   interfaceInputsParam")
    interfaceInputsParam = interfaceInputs.Param()

    print("   models")   
    models = mod.Models(simulation.param.simOptions.massModelName, allParam)

    print("   fsw")   
    fsw = fswModel.Fsw()

    print("================")
    print("Building parameters and states...")  
    # Simulation Parameters
    # ==================================
    print("   Simulation")
    simulation.build(patches)

    # Interface inputs parameters
    # ==================================
    print("   Interface inputs")
    interfaceInputsParam.keyboardInputsParam = paramTools.patchAttributes(interfaceInputsParam.keyboardInputsParam, patches["ifKeyboardParamPatch"])
    joystickSerialPort = interfaceInputsParam.joystickParam.initializeSerialPort()

    print("   Models")
    models.build(patches)
    
    # FSW
    # ==================================
    print("   FSW")
    # [TBW]
    fsw.build(patches, models)
    # [TBW] 
    fsw.run(simulation, models.interface)
    
    # [Displays]
    # ==================================
    print("   Displays")
    displays = 0
    # displays = display.Displays()
    #displays.initialize(simulation.param, modelsBus, fswBus, simBus, fswModeMgtState)
    
    # Get end time and duration
    simuEndTime = time.time()
    simuDuration = simuEndTime - simuStrTime
    print("Duration: " + str(simuDuration))

    return (simulation, fsw, models, interfaceInputsParam, joystickSerialPort, displays)
