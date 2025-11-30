
from dataclasses import dataclass

import numpy as np

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# DICTIONNARIES
# --------------------------------------------------
# Dictionnary from RW unit states to IDs
RW_UNIT_STATES = {}
RW_UNIT_STATES["OFF"] = 0
RW_UNIT_STATES["ON"] = 1

# Dicitonnary from IDs to RW unit states
RW_UNIT_STATES_ID = {}
for key in RW_UNIT_STATES.keys():
    rwUnitStateNb = RW_UNIT_STATES[key]
    RW_UNIT_STATES_ID[rwUnitStateNb] = key

# --------------------------------------------------
# DATA CLASSES
# --------------------------------------------------
# RW set model parameters
@dataclass
class RwSetParam:
    name: str
    id: int
    nbUnits: int
    dir_B: list
    inertia: list

# RW unit model parameters
@dataclass
class RwUnitParam:
    name: str
    id: str
    branch: str
    dir_B: list
    inertia: float

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# RW model
class Rw:
    def __init__(self, allRwSetParam):
        self.param = allRwSetParam
        self.nbSets = 0
        self.sets = {}
        for key in allRwSetParam.keys():
            rwSetParam = allRwSetParam[key]
            rwSetParam = RwSetParam(**rwSetParam)
            self.addSet(rwSetParam)

    def addSet(self, rwSetParam):
        self.nbSets += 1
        self.sets.update({rwSetParam.name: RwSet(rwSetParam)})

    def computeTrq(self, fswCmdBus):
        # [TBW]
        torqueCmd_B = np.array(fswCmdBus.signals["torqueCmdRw_B"].value)
        torque_B = torqueCmd_B
        return torque_B

# RW set model
class RwSet():
    def __init__(self, rwSetParam):
        self.param = rwSetParam
        self.units = {}
        self.state = RW_UNIT_STATES["ON"]
        for idUnit in range(1, rwSetParam.nbUnits+1):
            rwUnitParam = self.getRwUnitParam(idUnit)
            self.units.update({rwUnitParam.name: RwUnit(rwUnitParam)})

    def getRwUnitParam(self, idUnit):
        name = ("unit"+str(idUnit))
        branch = self.param.name
        dir_B = self.param.dir_B[idUnit-1]
        inertia = self.param.inertia[idUnit-1]
        rwUnitParamDict = {"name": name, "branch": branch, "id": idUnit, "dir_B": dir_B, "inertia": inertia}
        return RwUnitParam(**rwUnitParamDict)

    def getUnit(self, idUnit):
        name = ("unit"+str(idUnit))
        return self.units[name]

    def switchUnitOn(self, idUnit):
        name = ("unit"+str(idUnit))
        self.units[name].switchOn()

    def switchUnitOff(self, idUnit):
        name = ("unit"+str(idUnit))
        self.units[name].switchOff()

    def switchOn(self):
        self.state = RW_UNIT_STATES["ON"]
        for key in self.units:
            self.units[key].switchOn()

    def switchOff(self):
        self.state = RW_UNIT_STATES["OFF"]
        for key in self.units:
            self.units[key].switchOff()

    def computeTrq(self, fswCmdBus):
        torqueCmd_B = np.array(fswCmdBus.signals["torqueCmdRw_B"].value)
        torque_B = torqueCmd_B
        return torque_B

# RW unit model
class RwUnit():
    def __init__(self, rwUnitParam):
        self.param = rwUnitParam
        self.state = RW_UNIT_STATES["ON"]

    def switchOn(self):
        self.state = RW_UNIT_STATES["ON"]

    def switchOff(self):
        self.state = RW_UNIT_STATES["OFF"]

    def computeTrq(self, fswCmdBus):
        torqueCmd_B = np.array(fswCmdBus.signals["torqueCmdRw_B"].value)
        torque_B = torqueCmd_B
        return torque_B
