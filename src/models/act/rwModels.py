
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
# Reaction wheels model
class Rw:
    def __init__(self, allRwSetParam):
        # Parameters
        self.param = allRwSetParam
        # Sets
        self.nbSets = 0
        self.sets = {}
        for key in allRwSetParam.keys():
            rwSetParam = allRwSetParam[key]
            rwSetParam = RwSetParam(**rwSetParam)
            self.addSet(rwSetParam)
        # Interface
        self.interface = {"torque_B": np.array([0, 0, 0])}

    def addSet(self, rwSetParam):
        self.nbSets += 1
        self.sets.update({rwSetParam.name: RwSet(rwSetParam)})

    # Build
    def build(self, patches):
        # [TBW] To be implemented
        dum = 0

    # Run
    def run(self, simOptions, fswIf):
        # Fetch interface
        fswCmdIf = fswIf["command"]
        # Process
        if simOptions.isRwAct:
            torqueRw_B = self.computeTrq(fswCommands)
        else:
            torqueRw_B = np.array([0, 0, 0])
        # Update interface
        self.interface["torque_B"] = torqueRw_B

    def computeTrq(self, fswCmdIf):
        # Fetch interface
        torqueCmd_B = np.array(fswCmdIf["torqueCmdRw_B"])
        # Process
        torque_B = torqueCmd_B
        return torque_B


# RW set model
class RwSet():
    def __init__(self, rwSetParam):
        # Parameters
        self.param = rwSetParam
        # State
        self.state = RW_UNIT_STATES["ON"]
        # Units
        self.units = {}
        for idUnit in range(1, rwSetParam.nbUnits+1):
            rwUnitParam = self.getRwUnitParam(idUnit)
            self.units.update({rwUnitParam.name: RwUnit(rwUnitParam)})
        # Interface
        self.interface = {}

    # Build
    def build(self):
        # [TBW] To be implemented
        dum = 0

    # Run
    def run(self):
        # [TBW] To be implemented
        dum = 0

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

    # Not used for now
    def computeTrq(self, fswCmdIf):
        # # Fetch interface
        torqueCmd_B = fswCmdIf["torqueCmdRw_B"]
        # Process
        torque_B = torqueCmd_B
        return torque_B


# RW unit model
class RwUnit():
    def __init__(self, rwUnitParam):
        # Parameters
        self.param = rwUnitParam
        # State
        self.state = RW_UNIT_STATES["ON"]
        # Interface
        self.interface = {}

    # Run
    def run(self):
        # [TBW] To be implemented
        dum = 0

    def switchOn(self):
        self.state = RW_UNIT_STATES["ON"]

    def switchOff(self):
        self.state = RW_UNIT_STATES["OFF"]

    def computeTrq(self, fswCmdBus):
        torqueCmd_B = np.array(fswCmdBus.signals["torqueCmdRw_B"].value)
        torque_B = torqueCmd_B
        return torque_B

    # Not used for now
    def computeTrq(self, fswCmdIf):
        # Fetch interface
        torqueCmd_B = fswCmdIf["torqueCmdThr_B"]
        # Compute torque
        torque_B = torqueCmd_B
        return torque_B


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
# [TBW] Consumption model


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------