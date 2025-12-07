

import numpy as np

import ephem

import src.utils.constants as const
import src.utils.conversions as conversions

import src.models.dyn.orbitDynamics as orbDyn
import src.models.kin.attitudeKinematics as attKin
import src.models.dyn.attitudeDynamics as attDyn

from src.models.kin.attitudeKinematics import Rx as Rx
from src.models.kin.attitudeKinematics import Ry as Ry
from src.models.kin.attitudeKinematics import Rz as Rz

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# DATA CLASSES
# --------------------------------------------------


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Environment():
    def __init__(self):
        self.ephemerides = Ephemerides()
        self.aerodynamics = Aerodynamics()
        self.gravity = Gravity()


    def run(self, simParam, attDynParam, simBus, modelsBus):
        # Initialize output bus
        modelsEnvBus = modelsBus.subBuses["environment"]
        modelsEnvBusOut = modelsEnvBus
        # Run ephemerides model
        modelsEnvBusOut = self.ephemerides.run(simParam, simBus, modelsEnvBus)
        # Run gravity model
        modelsEnvBusOut = self.gravity.run(simParam, attDynParam, modelsBus)
        # Run aerodynamics model
        modelsEnvBusOut = self.aerodynamics.run(simParam, attDynParam, modelsEnvBus)
        # Compute external torque
        modelsEnvBusOut = self.computeTotExtTorque(modelsEnvBusOut)
        return modelsEnvBusOut


    # Total total external torque
    def computeTotExtTorque(self, modelsEnvBus):
        # Retrieve useful inputs
        torqueAero_B = modelsEnvBus.signals["torqueAero_B"].value
        torqueGG_B = modelsEnvBus.signals["torqueGG_B"].value
        # Initialize output bus
        modelsEnvBusOut = modelsEnvBus
        # Compute total external torque
        torqueExt_B = torqueAero_B + torqueGG_B
        # Update output bus signals
        modelsEnvBusOut.signals["torqueExt_B"].update(torqueExt_B)
        return modelsEnvBusOut


class Ephemerides():
    def __init__ (self):
        self.param = 0


    def run(self, simParam, simBus, modelsEnvBus):
        # Initialize outputs bus
        modelsEnvBusOut = modelsEnvBus
        # Retrieve inputs / parameters
        elapsedTime = simBus.signals["elapsedTime"].value
        dateTimeStart = simParam.dateTimeStart
        ephemEpoch = simParam.ephemEpoch
        # Compute ephemerides
        currDateTime = ephem.date(dateTimeStart + elapsedTime/(24*3600))
        marsEphem = ephem.Mars()
        marsEphem.compute(currDateTime, epoch = ephemEpoch)
        # Compute directions
        sunToMarsDir_I = conversions.latLonToCartesian(marsEphem.hlat/deg2rad, marsEphem.hlon/deg2rad, 1)
        # Update output bus signals
        modelsEnvBusOut.signals["sunToMarsDir_I"].update(sunToMarsDir_I)
        return modelsEnvBusOut        


class Aerodynamics():
    def __init__ (self):
        self.param = 0


    def run(self, simParam, attDynParam, modelsEnvBus):
        # Initialize outputs bus
        modelsEnvBusOut = modelsEnvBus
        # Compute aerodynamic storque
        torqueAero_B = computeAeroTorque()
        # Update output bus signals
        modelsEnvBusOut.signals["torqueAero_B"].update(torqueAero_B)
        return modelsEnvBusOut



class Gravity():
    def __init__ (self):
        self.param = 0

    def run(self, simParam, attDynParam, modelsBus):
        # Initialize outputs bus
        modelsEnvBusOut = modelsBus.subBuses["environment"]
        # Retrieve signals / parameters of interest
        qLI_sca = modelsBus.subBuses["environment"].signals["qLI_sca"].value
        qLI_vec = modelsBus.subBuses["environment"].signals["qLI_vec"].value
        qBI_sca = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].value
        qBI_vec = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].value
        angRate_BI_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].value
        sma = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["sma"].value
        mu = const.earthMu # TBW => use single common parameter for gravity model config
        orbitPulse = orbDyn.getOrbitRate(mu, sma)
        # Reconstruct the dynamic state and LVLH frame quaternion
        qLI = attKin.Quaternion(qLI_sca, qLI_vec)
        qBI = attKin.Quaternion(qBI_sca, qBI_vec)
        # Compute gravity gradient disturbance torque in body frame
        inertiaSc_B = attDynParam.inertiaSc_B
        if simParam.simOptions.isGGTorqueEnabled:
            torqueGG_B = computeGGTorque(inertiaSc_B, orbitPulse, qBI, qLI)
        else:
            torqueGG_B = np.array([0, 0, 0])
        # Update output bus signals
        modelsEnvBusOut.signals["torqueGG_B"].update(torqueGG_B)
        return modelsEnvBusOut
 


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def computeLvlhFrame(modelsBus):
    # Initialize output
    modelsBusOut = modelsBus
    # Retrieve signals / parameters of interest
    inc  = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["inc"].value
    raan = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["raan"].value
    argPer = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["argPer"].value
    ta = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["ta"].value
    sma = modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["sma"].value
    mu = const.earthMu # TBW => use single common parameter for gravity model config
    # Compute attitude
    mat = np.matmul(Rx(inc), Rz(raan))
    mat = np.matmul(Rz(argPer+ta), mat)
    mat = np.matmul(Ry(-np.pi/2), mat)
    dcmLI = np.matmul(Rz(np.pi/2), mat)
    qLI = attKin.trans_DcmToQuat(dcmLI).normalize()
    eulerAng_LI = qLI.toEuler()
    # Compute angular rates
    angRate_LI_L = np.array([0, -orbDyn.getOrbitRate(mu, sma), 0])
    # Update output bus signals
    modelsBusOut.subBuses["environment"].signals["eulerAng_LI"].update(eulerAng_LI)
    modelsBusOut.subBuses["environment"].signals["qLI_sca"].update(qLI.sca)
    modelsBusOut.subBuses["environment"].signals["qLI_vec"].update(qLI.vec)
    modelsBusOut.subBuses["environment"].signals["angRate_LI_L"].update(angRate_LI_L)
    return modelsBusOut


def computeGGTorque(inertiaSc_B, orbitPulse, qBI, qLI):
    dcmLI = qLI.toDcm()
    # Compute the Z axis of the LVLH frame in body frame
    zL_L = np.array([0, 0, 1])
    zL_I = np.matmul(np.transpose(dcmLI), zL_L)
    zL_B = attKin.applyRotation(qBI, zL_I)
    # Compute gravity gradient torque
    torqueGG_B = 3*pow(orbitPulse,2) * np.matmul(attKin.trans_VecToCrossMat(zL_B), np.matmul(inertiaSc_B, zL_B))
    return torqueGG_B


def computeAeroTorque():
    # TBW
    return np.array([0, 0, 0])


# --------------------------------------------------
# SIMULATION / TEST
# -------------------------------------------------- 