
from src.utils.runLoop import *
from src.utils.runPlots import *
from src.utils.runExport import *
from src.utils.runInitialization import *

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions


def runSimu(scenarioPatchFcnHdl):
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
    
    
    # --------------------------------------------------
    # PLOTS
    # --------------------------------------------------
    print("==================================")
    print("Plots")
    runPlots(modelsBus, fswBus)


    # --------------------------------------------------
    # EXPORT DATA
    # --------------------------------------------------
    print("==================================")
    print("Export data")
    runExport(modelsBus, fswBus, simParam)
    

    return (modelsBus, fswBus, simBus, simParam)
