
from dataclasses import dataclass

import numpy as np

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# DICTIONNARIES
# --------------------------------------------------
# Dictionnary from THR unit states to IDs
THR_UNIT_STATES = {}
THR_UNIT_STATES["OFF"] = 0
THR_UNIT_STATES["ON"] = 1

# Dicitonnary from IDs to THR unit states
THR_UNIT_STATES_ID = {}
for key in THR_UNIT_STATES.keys():
    thrUnitStateNb = THR_UNIT_STATES[key]
    THR_UNIT_STATES_ID[thrUnitStateNb] = key

# --------------------------------------------------
# DATA CLASSES
# --------------------------------------------------
# THR set model parameters
@dataclass
class ThrSetParam:
    name: str
    id: int
    nbUnits: int
    dir_B: list
    pos_B: list
    force: list

# THR unit model parameters
@dataclass
class ThrUnitParam:
    name: str
    id: str
    branch: str
    dir_B: list
    pos_B: list
    force: float

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# THR model
class Thr:
    def __init__(self, allThrSetParam):
        self.param = allThrSetParam
        self.nbSets = 0
        self.sets = {}
        for key in allThrSetParam.keys():
            thrSetParam = allThrSetParam[key]
            thrSetParam = ThrSetParam(**thrSetParam)
            self.addSet(thrSetParam)


    def addSet(self, thrSetParam):
        self.nbSets += 1
        self.sets.update({thrSetParam.name: ThrSet(thrSetParam)})


    def run(self, simOptions, fswCmdBus, modelsActBus):
        # Initialize outputs bus
        modelsActBusOut = modelsActBus
        # Process
        if simOptions.isThrAct:
            torqueThr_B = self.computeTrq(fswCmdBus)
        else:
            torqueThr_B = np.array([0, 0, 0])
        # Set outputs bus
        modelsActBusOut.signals["torqueThr_B"].update(torqueThr_B)
        return modelsActBusOut


    def computeTrq(self, fswCmdBus):
        # [TBW]
        torqueCmd_B = np.array(fswCmdBus.signals["torqueCmdThr_B"].value)
        torque_B = torqueCmd_B
        return torque_B


# THR set model
class ThrSet():
    def __init__(self, thrSetParam):
        self.param = thrSetParam
        self.units = {}
        self.state = THR_UNIT_STATES["ON"]
        for idUnit in range(1, thrSetParam.nbUnits+1):
            thrUnitParam = self.getThrUnitParam(idUnit)
            self.units.update({thrUnitParam.name: ThrUnit(thrUnitParam)})

    def getThrUnitParam(self, idUnit):
        name = ("unit"+str(idUnit))
        branch = self.param.name
        dir_B = self.param.dir_B[idUnit-1]
        pos_B = self.param.pos_B[idUnit-1]
        force = self.param.force[idUnit-1]
        thrUnitParamDict = {"name": name, "branch": branch, "id": idUnit, "dir_B": dir_B, "pos_B": pos_B, "force": force}
        return ThrUnitParam(**thrUnitParamDict)

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
        self.state = THR_UNIT_STATES["ON"]
        for key in self.units:
            self.units[key].switchOn()

    def switchOff(self):
        self.state = THR_UNIT_STATES["OFF"]
        for key in self.units:
            self.units[key].switchOff()

    def computeTrq(self, fswCmdBus):
        torqueCmd_B = np.array(fswCmdBus.signals["torqueCmdThr_B"].value)
        torque_B = torqueCmd_B
        return torque_B


# THR unit model
class ThrUnit():
    def __init__(self, thrUnitParam):
        self.param = thrUnitParam
        self.state = THR_UNIT_STATES["ON"]

    def switchOn(self):
        self.state = THR_UNIT_STATES["ON"]

    def switchOff(self):
        self.state = THR_UNIT_STATES["OFF"]

    def computeTrq(self, fswCmdBus):
        torqueCmd_B = np.array(fswCmdBus.signals["torqueCmdThr_B"].value)
        torque_B = torqueCmd_B
        return torque_B

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
# [TBW] Consumption model



# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------