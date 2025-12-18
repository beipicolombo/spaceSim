
import time


def runLoop(simulation, fsw, models, interfaceInputsParam, joystickSerialPort, displays):
    # Get start time
    simuStrTime = time.time()
    # Initial events list
    allEvtList = []

    for ii in range(1, simulation.param.nbPts):    
        # time.sleep(simulation.param.Ts)
        
        # [Simulation]
        # ==================================
        simulation.run()

        # [Models]
        # ==================================
        models.run(simulation, fsw.interface)

        # [FSW]
        # ==================================
        fsw.run(simulation, models.interface)

        # Update event management
        allEvtList = allEvtList + fsw.interface["modeMgt"]["events"]

        # [Graphics]
        # ==================================    
        # displays.update(simulation.param, modelsBus, fswBus, simBus, fswModeMgtState, qPrevLI, qPrevBI)

    joystickSerialPort.close()

    # Get end time and duration
    simuEndTime = time.time()
    simuDuration = simuEndTime - simuStrTime
    print("   Duration: " + str(simuDuration))

    return (simulation, fsw, models, allEvtList)