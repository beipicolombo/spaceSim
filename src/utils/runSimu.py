
from src.utils.runLoop import *
from src.utils.runPlots import *
from src.utils.runExport import *
from src.utils.runInitialization import *


def runSimu(scenarioPatchFcnHdl):
    # --------------------------------------------------
    # INITIALIZATION
    # --------------------------------------------------
    print("==================================")
    print("Initialization")
    (simulation, fsw, models, interfaceInputsParam, joystickSerialPort, displays) = runInitialization(scenarioPatchFcnHdl)


    # --------------------------------------------------
    # SIMULATION
    # --------------------------------------------------
    print("==================================")
    print("Simulation")
    (simulation, fsw, models, allEvtList) = runLoop(simulation, fsw, models, interfaceInputsParam, joystickSerialPort, displays)


    # --------------------------------------------------
    # POST-PROCESSING
    # --------------------------------------------------
    print("==================================")
    print("Post-processing")
    # Convert the events list into a dataframe
    allEvtDataframe = eventsAndTmTc.eventListToDataFrame(allEvtList)

    
    # --------------------------------------------------
    # PLOTS
    # --------------------------------------------------
    print("==================================")
    print("Plots")
    # runPlots(modelsBus, fswBus, simParam)


    # --------------------------------------------------
    # EXPORT DATA
    # --------------------------------------------------
    print("==================================")
    print("Export data")
    dictDataExport = 0
    # dictDataExport = runExport(modelsBus, fswBus, simulation.param)
    
    # Temporary
    simBus = []
    modelsBus = []
    fswBus = []
    # Set outputs dictionnary
    outputs = {"dictDataExport": dictDataExport,
               "simParam": simulation.param,
               "modelsBus": modelsBus,
               "fswBus": fswBus,
               "simBus": simBus, 
               "allEvtDataframe": allEvtDataframe}

    return outputs
