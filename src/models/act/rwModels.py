
import numpy as np

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# Reaction wheels model parameters
class Rw:
    def __init__(self):
        self.nbSets = 0
        self.sets = {}

    def addSet(self, setName, nbUnits):
        setId = self.nbSets + 1
        self.nbSets += 1
        self.sets.update({setName: RwSet(name = setName, id = setId, nbUnits = nbUnits)})

    def computeTrq(self, fswCmdBus):
        torqueCmd_B = np.array(fswCmdBus.signals["torqueCmdThr_B"].value)
        torque_B = np.array([0, 0, 0])
        if (self.nbSets > 0):
            for key in self.sets.keys():
                torque_B = torque_B + self.set[key].computeTrq(torqueCmd_B)/self.nbSets 
        return torque_B

# RW set model
class RwSet:
    def __init__(self, name = "empty", id = 0, nbUnits = 0):
        self.name = name
        self.id = id
        self.nbUnits = nbUnits
        self.units = {}
        for kk in range(1, nbUnits+1):
            unitName = ("unit"+str(kk))
            self.units.update({unitName: RwUnit(name = unitName, id = kk)})

    def computeTrq(self, torqueCmd_B):
        torque_B = torqueCmd_B
        return torque_B

# RW unit parameters
class RwUnit():
    def __init__(self, name = "empty", id = 0):
        self.name = name
        self.id = id
        self.posUnit_G = np.array([0, 0, 0]) # [-]
        self.dirUnit_G = np.array([0, 0, -1]) # [-]
        self.inertia = 0 # [kg*m^2]