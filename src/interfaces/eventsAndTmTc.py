
import numpy as np
import pandas as pd

# Class for telecommands
class Tc():
    def __init__(self, name = "NULL", id = -1, time = 0):
        self.name = name
        self.id = id
        self.time = time

    def existsInDatabase(self, tcDatabase):
        return (any(tcId == self.id for tcId in tcDatabase.idList))

    def isEmpty(self):
        return (self.id == -1)


# Class for telecommands database
class TcDatabase():
    def __init__(self):
        self.dict = {}
        self.idList = []
        self.nameList = []

    def addTc(self, tcToAdd):
        self.dict.update({tcToAdd.name: tcToAdd})
        self.idList.append(tcToAdd.id)
        self.nameList.append(tcToAdd.name)

    def getTc(self, tcName, time):
        tcOut = self.dict[tcName]
        tcOut.time = time
        return tcOut


# Class for events
class Event():
    def __init__(self, name = "NULL", id = -1, time = 0):
        self.name = name
        self.id = id
        self.time = time

    def existsInDatabase(self, eventDatabase):
        return (any(eventId == self.id for eventId in eventDatabase.idList))

    def display(self):
        print("EVENT at elapsed time: " + str(self.time) + " [s] " + self.name + " ID: " + str(self.id))


# Function to transform a list of events into an exploitable dataframe
def eventListToDataFrame(evtList):
    data = {}
    data["TIME [s]"] = []
    data["NAME"] = []
    data["ID"] = []

    for evt in evtList:
        data["TIME [s]"].append(evt.time)
        data["NAME"].append(evt.name)
        data["ID"].append(evt.id)
    df = pd.DataFrame(data)
    return df


# Class for events database
class EventDatabase():
    def __init__(self):
        self.dict = {}
        self.idList = []
        self.nameList = []

    def addEvent(self, eventToAdd):
        self.dict.update({eventToAdd.name: eventToAdd})
        self.idList.append(eventToAdd.id)
        self.nameList.append(eventToAdd.name)

    def getEvent(self, evtName, time):
        evtOut = self.dict[evtName]
        evtOut.time = time
        return evtOut


# Function for TC listener
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


