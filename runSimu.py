# -*- coding: utf-8 -*-
"""
Created on Fri Jul 19 14:44:02 2024

@author: Xela
"""

import time
import numpy as np

from src.utils.runLoop import *
from src.utils.runPlots import *

import src.utils.initialization as initialization
import src.utils.constants as const
import src.data.scenarios.scenarioDefinitions as scenarioDefinitions


# Retrieve useful constants
pi  = np.pi
deg2rad = const.deg2rad
earthRadius = const.earthRadius
earthMu = const.earthMu


# --------------------------------------------------
# SETUP
# --------------------------------------------------
print("==================================")
print("Setup")
# Patches
patches = scenarioDefinitions.scenarioDefinition_testDevelopment()

# Initialization
(simParam, interfaceInputsParam, fswParam, scParam, fswModeMgtState, modelsBus, fswBus, simBus, joystickSerialPort, displays) = initialization.initializeSimulation(patches)


# --------------------------------------------------
# SIMULATION
# --------------------------------------------------
print("==================================")
print("Simulation")
(modelsBus, fswBus, simBus) = runLoop(simParam, interfaceInputsParam, fswParam, scParam, fswModeMgtState, modelsBus, fswBus, simBus, joystickSerialPort, displays)

    
# --------------------------------------------------
# DEBUG
# --------------------------------------------------


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



