# -*- coding: utf-8 -*-
"""
Created on Fri Jul 19 14:44:02 2024

@author: Xela
"""

import time
import numpy as np

from src.utils.runLoop import *
from src.utils.runPlots import *
from src.utils.runInitialization import *

import src.utils.constants as const
import src.data.scenarios.scenarioDefinitions as scenarioDefinitions


# Retrieve useful constants
pi  = np.pi
deg2rad = const.deg2rad
earthRadius = const.earthRadius
earthMu = const.earthMu

# Define the scenario name
scenarioPatchFcnHdl = scenarioDefinitions.scenarioDefinition_testDevelopment


# --------------------------------------------------
# INITIALIZATION
# --------------------------------------------------
print("==================================")
print("Initialization")
(simParam, interfaceInputsParam, fswParam, scParam, fswModeMgtState, modelsBus, fswBus, simBus, joystickSerialPort, displays) = runInitialization(scenarioPatchFcnHdl)


# --------------------------------------------------
# SIMULATION
# --------------------------------------------------
print("==================================")
print("Simulation")
(modelsBus, fswBus, simBus) = runLoop(simParam, interfaceInputsParam, fswParam, scParam, fswModeMgtState, modelsBus, fswBus, simBus, joystickSerialPort, displays)


# --------------------------------------------------
# POST-PROCESSING
# --------------------------------------------------
print("==================================")
print("Post-processing")
runPlots(modelsBus, fswBus)


# --------------------------------------------------
# PLOTS
# --------------------------------------------------
print("==================================")
print("Plots")
runPlots(modelsBus, fswBus)



