
import time

import src.interfaces.interfaceInputs as interfaceInputs
import src.interfaces.eventsAndTmTc as eventsAndTmTc

import src.models.dyn.attitudeDynamics as attDyn
import src.models.dyn.orbitDynamics as orbDyn
import src.models.kin.attitudeKinematics as attKin

import src.models.spaceEnv.envModel as envModel
import src.models.spaceEnv.perfModel as perfModel


def runLoop(simParam, interfaceInputsParam, fsw, models, attitude, orbit, modelsBus, fswBus, simBus, joystickSerialPort, displays):
    
    simuStrTime = time.time()
    tcList = simParam.tcTimeline # list of TC to be received
    allEvtList = []

    for ii in range(1, simParam.nbPts):    
        # time.sleep(simParam.Ts)
        
        # [Simulation]
        # ==================================
        # Update time
        elapsedTimePrev = simBus.signals["elapsedTime"].value
        simBus.signals["elapsedTime"].update(elapsedTimePrev + simParam.Ts)

        # [Models]
        # ==================================    
        # Environment
        modelsBus = models.run(simParam, simBus, fswBus, modelsBus)
        
        # Run attitude model
        modelsBus = attitude.run(simParam, models.spacecraft, modelsBus)

        # Run orbit model
        modelsBus = orbit.run(simParam, modelsBus)

        # Sensor
        modelsBus.subBuses["sensors"] = models.spacecraft.sen.computeMeasurements(modelsBus.subBuses["sensors"], modelsBus)

        # LVLH frame
        modelsBus = envModel.computeLvlhFrame(modelsBus)

        # Performance (additional models signals)
        modelsBus = perfModel.computeBodyNadirAttitude(modelsBus)

        # [FSW]
        # ==================================
        # Retrieve received TCs
        (receivedTcList, tcList) = eventsAndTmTc.listenTc(simBus, tcList)

        # [Mode management]
        (fswBus.subBuses["modeMgt"], modeMgtEvtList) = fsw.modeMgt.run(simParam, fswBus, simBus, receivedTcList)
        allEvtList = allEvtList + modeMgtEvtList

        # [Interfaces]
        fswBus.subBuses["interfaces"] = interfaceInputs.getInterfaceInputs(fswBus, simBus, joystickSerialPort, interfaceInputsParam)

        # [Sensor processing]
        fswBus.subBuses["sensorProc"] = fsw.sensorProc.run(fswBus, modelsBus)

        # [Estimation]
        fswBus.subBuses["estimation"] = fsw.estimation.run(fswBus, modelsBus)

        # [Guidance]
        fswBus.subBuses["guidance"] = fsw.guidance.run(fswBus)

        # [Control]
        fswBus.subBuses["control"] = fsw.control.run(fswBus)

        # [Command]
        fswBus.subBuses["command"] = fsw.command.run(fswBus)

        # [Graphics]
        # ==================================    
        # displays.update(simParam, modelsBus, fswBus, simBus, fswModeMgtState, qPrevLI, qPrevBI)

    # Convert the events list into a dataframe
    allEvtDataframe = eventsAndTmTc.eventListToDataFrame(allEvtList)

    simuEndTime = time.time()
    simuDuration = simuEndTime - simuStrTime
    print("   Duration: " + str(simuDuration))

    joystickSerialPort.close()
    
    return (modelsBus, fswBus, simBus, allEvtDataframe)