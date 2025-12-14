
import time

import src.interfaces.interfaceInputs as interfaceInputs
import src.interfaces.eventsAndTmTc as eventsAndTmTc

import src.models.dyn.attitudeDynamics as attDyn
import src.models.dyn.orbitDynamics as orbDyn
import src.models.kin.attitudeKinematics as attKin

import src.models.spaceEnv.envModel as envModel
import src.models.spaceEnv.perfModel as perfModel


def runLoop(simulation, interfaceInputsParam, fsw, models, modelsBus, fswBus, simBus, joystickSerialPort, displays):
    
    simuStrTime = time.time()
    tcList = simulation.param.tcTimeline # list of TC to be received
    allEvtList = []

    for ii in range(1, simulation.param.nbPts):    
        # time.sleep(simulation.param.Ts)
        
        # [Simulation]
        # ==================================
        # Update time
        simulation.run()

        # [Models]
        # ==================================
        models.run(simulation, fsw.interface)

        # Performance (additional models signals)
        # TBW
        # modelsBus = perfModel.computeBodyNadirAttitude(modelsBus)

        # [FSW]
        # ==================================
        # Retrieve received TCs
        (receivedTcList, tcList) = eventsAndTmTc.listenTc(simBus, tcList)

        fsw.run(simulation, models.interface, receivedTcList)

        # Update event management
        allEvtList = receivedTcList
        # allEvtList = allEvtList + modeMgtEvtList

        # [Graphics]
        # ==================================    
        # displays.update(simulation.param, modelsBus, fswBus, simBus, fswModeMgtState, qPrevLI, qPrevBI)

    # Convert the events list into a dataframe
    allEvtDataframe = eventsAndTmTc.eventListToDataFrame(allEvtList)

    simuEndTime = time.time()
    simuDuration = simuEndTime - simuStrTime
    print("   Duration: " + str(simuDuration))

    joystickSerialPort.close()
    
    return (modelsBus, fswBus, simBus, allEvtDataframe)