
from dataclasses import dataclass

import numpy as np


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# DICTIONNARIES
# --------------------------------------------------
# Dictionnary from IMU unit states to IDs
IMU_UNIT_STATES = {}
IMU_UNIT_STATES["OFF"] = 0
IMU_UNIT_STATES["ON"] = 1

# Dicitonnary from IDs to IMU unit states
IMU_UNIT_STATES_ID = {}
for key in IMU_UNIT_STATES.keys():
    imuUnitStateNb = IMU_UNIT_STATES[key]
    IMU_UNIT_STATES_ID[imuUnitStateNb] = key

# --------------------------------------------------
# DATA CLASSES
# --------------------------------------------------
# IMU set model parameters
@dataclass
class ImuSetParam:
    name: str
    id: int
    nbUnits: int
    dir_B: list
    pos_B: list

# IMU unit model parameters
@dataclass
class ImuUnitParam:
    name: str
    id: str
    branch: str
    dir_B: list
    pos_B: list

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# Inertial measurement units model
class Imu():
    def __init__(self, allImuSetParam):
        self.param = allImuSetParam
        self.nbSets = 0
        self.sets = {}
        for key in allImuSetParam.keys():
            imuSetParam = ImuSetParam(**allImuSetParam[key])
            self.addSet(imuSetParam)
        # Interface
        self.interface = {}
        self.interface.update({"angRateMeas_B": np.array([0, 0, 0])})

    def addSet(self, imuSetParam):
        self.nbSets += 1
        self.sets.update({imuSetParam.name: ImuSet(imuSetParam)})

    # Build
    def build(self, patches):
        # [TBW] To be implemented
        dum = 0

    # Run
    def run(self, dynIf, envIf):
        # Fetch dynamics and environment interfaces
        angRate_BI_B = dynIf["attitude"]["angRate_BI_B"]
        # Process
        angRateMeas_B = angRate_BI_B
        # Update interface
        self.interface["angRateMeas_B"] = angRateMeas_B
        

# IMU set model
class ImuSet():
    def __init__(self, imuSetParam):
        self.param = imuSetParam
        self.units = {}
        self.state = IMU_UNIT_STATES["ON"]
        for idUnit in range(1, imuSetParam.nbUnits+1):
            imuUnitParam = self.getImuUnitParam(idUnit)
            self.units.update({imuUnitParam.name: ImuUnit(imuUnitParam)})

    def getImuUnitParam(self, idUnit):
        name = ("unit"+str(idUnit))
        branch = self.param.name
        dir_B = self.param.dir_B[idUnit-1]
        pos_B = self.param.pos_B[idUnit-1]
        imuUnitParamDict = {"name": name, "branch": branch, "id": idUnit, "dir_B": dir_B, "pos_B": pos_B}
        return ImuUnitParam(**imuUnitParamDict)

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
        self.state = IMU_UNIT_STATES["ON"]
        for key in self.units:
            self.units[key].switchOn()

    def switchOff(self):
        self.state = IMU_UNIT_STATES["OFF"]
        for key in self.units:
            self.units[key].switchOff()


# IMU unit model
class ImuUnit():
    def __init__(self, imuUnitParam):
        self.param = imuUnitParam
        self.state = IMU_UNIT_STATES["ON"]

    def switchOn(self):
        self.state = IMU_UNIT_STATES["ON"]

    def switchOff(self):
        self.state = IMU_UNIT_STATES["OFF"]  

