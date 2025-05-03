
import numpy as np

class Tc():
    def __init__(self, name = "EMPTY", id = -1, time = 0):
        # TC_AOCS_MODE_SWITCH_SAFE_TO_NOM_PTNG (1)
        # TC_AOCS_MODE_SWITCH_NOM_PTNG_TO_NOM_EQ (2)
        self.name = name
        self.id = id
        self.time = time

    def isEmpty(self):
        return (self.id == -1)

class Event():
    def __init__(self, name = "NULL", id = -1, time = 0):
        # EVT_AOCS_MODE_SWITCH (1)
        # EVT_TC_RECEIVED (2)
        # EVT_TC_ACCEPTED (3)
        # EVT_TC_REJECTED (4)
        self.name = name
        self.id = id
        self.time = time

    def display(self):
        print("EVENT at elapsed time: " + str(self.time) + " [s] " + self.name + " ID: " + str(self.id))


def listenTc(simBus, tcList):
    # Get the current time
    currTime = simBus.signals["elapsedTime"].value

    # Initialize outputs
    updatedTcList = tcList
    receivedTcList = []

    if (len(tcList) > 0):
        # Check if the TC is received
        # TBW: handle if TCs are received at the same time
        if (currTime >= tcList[0].time):
            receivedTcList.append(updatedTcList.pop(0))

    return (receivedTcList, updatedTcList)