
import numpy as np

import src.utils.paramTools as paramTools

import src.models.kin.attitudeKinematics as attitudeKinematics

deg2rad = np.pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# Estimation function
class Estimation():
    def __init__(self):
        # Parameters
        self.param = Param()
        # State
        self.state = State()
        # Interface
        self.interface = {}
        self.interface.update({"posEst_J": np.array([0, 0, 0])})
        self.interface.update({"velEst_J": np.array([0, 0, 0])})
        self.interface.update({"qLIest": attitudeKinematics.Quaternion()})
        self.interface.update({"angRateEst_LI_L": np.array([0, 0, 0])})
        self.interface.update({"angRateEst_BI_B": np.array([0, 0, 0])})
        self.interface.update({"eulerAngEst_BI": np.array([0, 0, 0])})
        self.interface.update({"qBIest": attitudeKinematics.Quaternion()})

    # Build 
    def build(self, patches):
        # Do nothing for now
        # Patch parameters
        self.param = Param()
        # Set state
        self.state = State()

    # Run
    def run(self, fswModeMgtIf, fswSensorProcIf, modelsIf):
        # Position and velocity estimation
        self.positionVelocityEstimation(fswModeMgtIf, fswSensorProcIf, modelsIf)
        # Attitude estimation
        self.attitudeEstimation(fswModeMgtIf, fswSensorProcIf, modelsIf)


    def positionVelocityEstimation(self, fswModeMgtIf, fswSensorProcIf, modelsIf):
        # Fetch models interface
        pos_J = modelsIf["dynamics"]["orbit"]["pos_I"]
        vel_J = modelsIf["dynamics"]["orbit"]["vel_I"]
        qLI = modelsIf["environment"]["qLI"]
        angRate_LI_L = modelsIf["environment"]["angRate_LI_L"]
        # Position estimation
        posEst_J = pos_J
        # Velocity estimation
        velEst_J = vel_J
        # Nadir frame estimation
        qLIest = qLI
        angRateEst_LI_L = angRate_LI_L
        # Update interface
        self.interface["posEst_J"] = posEst_J
        self.interface["velEst_J"] = velEst_J
        self.interface["qLIest"] = qLIest
        self.interface["angRateEst_LI_L"] = angRateEst_LI_L


    def attitudeEstimation(self, fswModeMgtIf, fswSensorProcIf, modelsIf):
        # Fetch models interface
        angRateMeas_BI_B = modelsIf["spacecraft"]["sensors"]["imu"]["angRateMeas_B"]
        qBImeas = modelsIf["spacecraft"]["sensors"]["str"]["qBImeas"]
        # Angular rates estimation
        angRateEst_BI_B = angRateMeas_BI_B
        # Attitude estimation
        qBIest = qBImeas
        # Update interface        
        self.interface["qBIest"] = qBIest
        self.interface["angRateEst_BI_B"] = angRateEst_BI_B


# Estimation state
class State():
    def __init__(self):
        # [TBW] Dummy state
        self.dummy = 0


# Estimation parameters
class Param():
    def __init__(self):
        # [TBW] Dummy parameters
        self.dummy = 0


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------

