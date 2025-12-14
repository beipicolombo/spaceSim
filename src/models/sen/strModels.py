
from dataclasses import dataclass

import numpy as np
import src.models.kin.attitudeKinematics as attitudeKinematics


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# DICTIONNARIES
# --------------------------------------------------
# Dictionnary from STR unit states to IDs
STR_UNIT_STATES = {}
STR_UNIT_STATES["OFF"] = 0
STR_UNIT_STATES["ON"] = 1

# Dicitonnary from IDs to STR unit states
STR_UNIT_STATES_ID = {}
for key in STR_UNIT_STATES.keys():
    strUnitStateNb = STR_UNIT_STATES[key]
    STR_UNIT_STATES_ID[strUnitStateNb] = key

# --------------------------------------------------
# DATA CLASSES
# --------------------------------------------------
# STR set model parameters
@dataclass
class StrSetParam:
    name: str
    id: int
    nbUnits: int
    dir_B: list
    pos_B: list

# STR unit model parameters
@dataclass
class StrUnitParam:
    name: str
    id: str
    branch: str
    dir_B: list
    pos_B: list

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Str():
	def __init__(self, allStrSetParam):
		self.param = allStrSetParam
		self.nbSets = 0
		self.sets = {}
		for key in allStrSetParam.keys():
			strSetParam = StrSetParam(**allStrSetParam[key])
			self.addSet(strSetParam)

	def addSet(self, strSetParam):
		self.nbSets += 1
		self.sets.update({strSetParam.name: StrSet(strSetParam)})        

	def run(self, modelsBus):
        # Retrieve inputs
		qBI_sca     = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].value
		qBI_vec     = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].value
		eulerAng_BI = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].value
        # Process
		eulerAngMeas_BI = eulerAng_BI
		qBImeas_sca = qBI_sca
		qBImeas_vec = qBI_vec
		qBImeas = attitudeKinematics.Quaternion(qBImeas_sca, qBImeas_vec).normalize()
		return (qBImeas, eulerAngMeas_BI)


# STR set model
class StrSet():
    def __init__(self, strSetParam):
        self.param = strSetParam
        self.units = {}
        self.state = STR_UNIT_STATES["ON"]
        for idUnit in range(1, strSetParam.nbUnits+1):
            strUnitParam = self.getStrUnitParam(idUnit)
            self.units.update({strUnitParam.name: StrUnit(strUnitParam)})

    def getStrUnitParam(self, idUnit):
        name = ("unit"+str(idUnit))
        branch = self.param.name
        dir_B = self.param.dir_B[idUnit-1]
        pos_B = self.param.pos_B[idUnit-1]
        strUnitParamDict = {"name": name, "branch": branch, "id": idUnit, "dir_B": dir_B, "pos_B": pos_B}
        return StrUnitParam(**strUnitParamDict)

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
        self.state = STR_UNIT_STATES["ON"]
        for key in self.units:
            self.units[key].switchOn()

    def switchOff(self):
        self.state = STR_UNIT_STATES["OFF"]
        for key in self.units:
            self.units[key].switchOff()


# STR unit model
class StrUnit():
    def __init__(self, strUnitParam):
        self.param = strUnitParam
        self.state = STR_UNIT_STATES["ON"]

    def switchOn(self):
        self.state = STR_UNIT_STATES["ON"]

    def switchOff(self):
        self.state = STR_UNIT_STATES["OFF"]          