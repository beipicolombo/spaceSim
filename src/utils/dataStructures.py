# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np
import matplotlib.pyplot as plt


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# Data bus
class DataBus():
    def __init__(self, busName = "", busPath = ""):
        self.name = busName
        self.subBuses = {}
        self.signals = {}
        self.path = busPath
    
    def addSubBus(self, subBusName):
        subBusPath = (self.name + "/" + subBusName)
        self.subBuses[subBusName] = DataBus(busName = subBusName, busPath = subBusPath)
        
    def addSignal(self, signalName, signalSize, unit = "-", isLogged = False, timeVec = [], isExport = False):
        signalPath = (self.path + "/" + signalName)
        self.signals[signalName] = Signal(signalName, signalSize, unit, isLogged, timeVec, isExport, signalPath)

    def setSignalsLog(self, signalPathsToLog, timeVec):
        for path in signalPathsToLog:
            setSignalLog(self, path, timeVec)

    def setSignalsExport(self, signalPathsToExport, timeVec):
        for path in signalPathsToExport:
            setSignalExport(self, path, timeVec)
        
# Data signal
class Signal():
    def __init__(self, signalName = "", signalSize = 0, unit = "-", isLogged = False, timeVec = [], isExport = False, signalPath = ""):
        self.name = signalName
        self.size = signalSize
        self.isLogged = isLogged
        self.isExport = isExport
        self.unit = unit
        self.value = 0
        self.timeseries = Timeseries(timeVec, self.size, self.unit, self.name)
        self.path = signalPath

    def update(self, value):
        self.value = value
        if self.isLogged:
            self.timeseries.dataVec[self.timeseries.idx] = value
            self.timeseries.idx += 1

# Simulation log
class Timeseries:
    def __init__(self, timeVec, nComponents, unit, name):
        self.timeVec = timeVec
        self.nComponents = nComponents
        if nComponents>2:
            self.dataVec = np.zeros((len(timeVec), nComponents))
        else:
            self.dataVec = np.zeros(len(timeVec))
        self.idx = 0
        self.unit = unit
        self.name = name
            
    def store(self, ii, vecToLog):
        self.dataVec[ii,:] = vecToLog
        
    def m2km(self):
        self.dataVec = self.dataVec / 1e3
        self.unit = "km"
        return self

    def rad2deg(self):
        self.dataVec = self.dataVec / deg2rad
        if self.unit == "rad":
            self.unit = "deg"
        elif self.unit == "rad/s":
            self.unit = "deg/s"
        else:
            self.unit = "deg/s^2"
        return self

    def addNorm(self):
        dataVecNorm = np.reshape(np.linalg.norm(self.dataVec, axis = 1), (len(self.timeVec), 1))
        self.dataVec = np.concatenate((self.dataVec, dataVecNorm), axis = 1)
        self.nComponents += 1

        return self

    def getNorm(self):
        outputTs = Timeseries(self.timeVec, 1, self.unit, ("norm_" + self.name))
        outputTs.dataVec = np.linalg.norm(self.dataVec, axis = 1)
        
        return outputTs

    def deg2rad(self):      
        self.dataVec = self.dataVec * deg2rad
        if self.unit == "deg":
            self.unit = "rad"
        elif self.unit == "deg/s":
            self.unit = "rad/s"
        else:
            self.unit = "rad/s^2"
            
    def plot(self):
        print("   " + self.name)
        fig = plt.figure()
        plt.grid()
        for iiAx in range(self.nComponents):
            plt.plot(self.timeVec, self.dataVec[:,iiAx])
        plt.ylabel(self.unit)
        plt.xlabel("s")
        plt.title(self.name)
        
        return fig
        

    def toDic(self):
        dict = {}
        for ii in range(self.nComponents):
            axisName = self.name + "_" + str(ii)
            axisData = self.dataVec[:, ii]
            dict.update({axisName : axisData})

        return dict



def getSignalObjFromPath(myDataBus, path):
    sep = "/"
    pathSplit = path.split(sep)
    signalName = pathSplit[-1]
    subBusesNames = pathSplit[0:-1]

    if (myDataBus.name == pathSplit[0]):    
        if len(subBusesNames) == 1:
            signalObj = myDataBus.signals[signalName]
        else:
            redPath = sep.join(subBusesNames[1:])
            redPath = (redPath + sep + signalName)     
            signalObj = getSignalObjFromPath(myDataBus.subBuses[subBusesNames[1]], redPath)
            
    return signalObj


def setSignalLog(myDataBus, path, timeVec):
    sep = "/"
    pathSplit = path.split(sep)
    signalName = pathSplit[-1]
    subBusesNames = pathSplit[0:-1]

    if (myDataBus.name == pathSplit[0]):    
        if len(subBusesNames) == 1:
            myDataBus.signals[signalName].isLogged = True
            myDataBus.signals[signalName].timeseries.timeVec = timeVec
            myDataBus.signals[signalName].timeseries.dataVec = np.zeros((len(timeVec), myDataBus.signals[signalName].timeseries.nComponents))  
        else:
            redPath = sep.join(subBusesNames[1:])
            redPath = (redPath + sep + signalName)     
            setSignalLog(myDataBus.subBuses[subBusesNames[1]], redPath, timeVec)


def setSignalExport(myDataBus, path, timeVec):
    sep = "/"
    pathSplit = path.split(sep)
    signalName = pathSplit[-1]
    subBusesNames = pathSplit[0:-1]

    if (myDataBus.name == pathSplit[0]):    
        if len(subBusesNames) == 1:
            # Log automatically if it is not already the case
            if not(myDataBus.signals[signalName].isLogged):
                myDataBus.signals[signalName].isLogged = True
                myDataBus.signals[signalName].timeseries.timeVec = timeVec
                myDataBus.signals[signalName].timeseries.dataVec = np.zeros((len(timeVec), myDataBus.signals[signalName].timeseries.nComponents))  
            # Set the export parameter
            myDataBus.signals[signalName].isExport = True

        else:
            redPath = sep.join(subBusesNames[1:])
            redPath = (redPath + sep + signalName)     
            setSignalLog(myDataBus.subBuses[subBusesNames[1]], redPath, timeVec)