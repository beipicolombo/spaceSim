

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
# ...


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Environment():
    def __init__(self):
        self.ephemerides = Ephemerides()
        self.aerodynamics = Aerodynamics()
        self.gravity = Gravity()
        # Interface
        self.interface = {}
        self.interface.update({"torqueEnv_B": np.array([0, 0, 0])})
        self.interface.update({"qLI": attKin.Quaternion()})
        self.interface.update({"angRate_LI_L": np.array([0, 0, 0])})
        self.interface.update({"gravity": self.gravity.interface})
        self.interface.update({"ephemerides": self.ephemerides.interface})
        self.interface.update({"aerodynamics": self.aerodynamics.interface})


    def run(self, simParam, spacecraft, simIf, dynIf):
        # Compute LVLH frame
        self.computeLvlhFrame(dynIf)

        # Run ephemerides model
        self.ephemerides.run(simParam, simIf)

        # Run gravity model
        self.gravity.run(simParam, spacecraft, dynIf, self.interface)
        # Run aerodynamics model
        self.aerodynamics.run(simParam, spacecraft, dynIf, self.interface)
        
        # Compute external torque
        self.computeTotExtTorque()


    # Total total external torque
    def computeTotExtTorque(self):
        # Fetch gravity and aerodynamics interface
        torqueAero_B = self.aerodynamics.interface["torque_B"]
        torqueGG_B = self.gravity.interface["torque_B"]
        # Compute total external torque
        torqueExt_B = torqueAero_B + torqueGG_B
        # Update interface
        self.interface["torqueEnv_B"] = torqueExt_B


    # LVLH frame
    def computeLvlhFrame(self, dynIf):
        # Fetch dynamics interface
        inc  = dynIf["orbit"]["keplerElem"].inc
        raan = dynIf["orbit"]["keplerElem"].raan
        argPer = dynIf["orbit"]["keplerElem"].argPer
        ta = dynIf["orbit"]["keplerElem"].ta
        sma = dynIf["orbit"]["keplerElem"].sma
        # Retrieve parameter
        # TBW => use single common parameter for gravity model config
        mu = const.earthMu
        # Compute attitude
        mat = np.matmul(Rx(inc), Rz(raan))
        mat = np.matmul(Rz(argPer+ta), mat)
        mat = np.matmul(Ry(-np.pi/2), mat)
        dcmLI = np.matmul(Rz(np.pi/2), mat)
        qLI = attKin.trans_DcmToQuat(dcmLI).normalize()
        eulerAng_LI = qLI.toEuler()
        # Compute angular rates
        angRate_LI_L = np.array([0, -orbDyn.getOrbitRate(mu, sma), 0])
        # Update intercace
        self.interface["qLI"] = qLI
        self.interface["angRate_LI_L"] = angRate_LI_L


class Ephemerides():
    def __init__ (self):
        # TBW
        self.param = 0
        # Interface
        self.interface = {}
        self.interface.update({"sunToMarsDir_I": np.array([0, 0, 0])})

    def run(self, simParam, simIf):
        # Fetch simulation interface
        elapsedTime = simIf["elapsedTime"]
        # Compute epoch
        dateTimeStart = simParam.dateTimeStart
        ephemEpoch = simParam.ephemEpoch
        # Compute ephemerides
        currDateTime = ephem.date(dateTimeStart + elapsedTime/(24*3600))
        marsEphem = ephem.Mars()
        marsEphem.compute(currDateTime, epoch = ephemEpoch)
        # Compute directions
        sunToMarsDir_I = conversions.latLonToCartesian(marsEphem.hlat/deg2rad, marsEphem.hlon/deg2rad, 1)
        # Update interface
        self.interface["sunToMarsDir_I"] = sunToMarsDir_I


class Aerodynamics():
    def __init__ (self):
        # TBW
        self.param = 0
        # Interface
        self.interface = {"torque_B": np.array([0, 0, 0])}

    def run(self, simParam, spacecraft, dynIf, envIf):
        # Compute aerodynamic storque
        torqueAero_B = computeAeroTorque()
        # Update interface
        self.interface["torque_B"] = torqueAero_B


class Gravity():
    def __init__ (self):
        # TBW
        self.param = 0
        # Interface
        self.interface = {"torqueGG_B": np.array([0, 0, 0])}

    def run(self, simParam, spacecraft, dynIf, envIf):
        # Fetch environment and dynamics interface
        qLI = envIf["qLI"]
        angRate_LI_L = envIf["angRate_LI_L"]
        
        qBI = dynIf["attitude"]["qBI"]
        angRate_BI_B = dynIf["attitude"]["angRate_BI_B"]

        sma = dynIf["orbit"]["keplerElem"].sma

        # Gravity parameter
        # TBW => use single common parameter for gravity model config
        mu = const.earthMu 

        # Compute orbit pulse
        orbitPulse = orbDyn.getOrbitRate(mu, sma)

        # Compute gravity gradient disturbance torque in body frame
        inertiaSc_B = spacecraft.massModel.inertiaSc_B
        if simParam.simOptions.isGGTorqueEnabled:
            torqueGG_B = computeGGTorque(inertiaSc_B, orbitPulse, qBI, qLI)
        else:
            torqueGG_B = np.array([0, 0, 0])
        # Update interface
        self.interface["torque_B"] = torqueGG_B
 


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
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