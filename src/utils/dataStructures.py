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
    def __init__(self, busName = ""):
        self.name = busName
        self.subBuses = {}
        self.signals = {}
    
    def addSubBus(self, busName):
        self.subBuses[busName] = DataBus(busName)
        
    def addSignal(self, signalName, signalSize, unit = "-", isLogged = False, timeVec = []):
        self.signals[signalName] = Signal(signalName, signalSize, unit, isLogged, timeVec)

# Data signal
class Signal():
    def __init__(self, signalName = "", signalSize = 0, unit = "-", isLogged = False, timeVec = []):
        self.name = signalName
        self.size = signalSize
        self.isLogged = isLogged
        self.unit = unit
        self.value = 0
        self.timeseries = Timeseries(timeVec, self.size, self.unit, self.name)

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
        self.dataVec = np.zeros((len(timeVec), nComponents))
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
        plt.show()