
import numpy as np
import src.models.kin.attitudeKinematics as attKin

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
# WIP
class Attitude():
    def __init__(self):
        self.state = State()
        self.config = Config()
        # Interface
        self.interface = {}
        self.interface.update({"qBI": attKin.Quaternion()})
        self.interface.update({"angRate_BI_B": np.array([0, 0, 0])})
        self.interface.update({"angMomSc_B": np.array([0, 0, 0])})


    def set(self, simParam, spacecraft, envIf):
        # Fetch environment interface
        # LVLH frame: temp
        qLI = envIf["qLI"]
        angRate_LI_L = envIf["angRate_LI_L"]

        # Retrieve inputs
        qInit = self.config.qInit
        angRateInit_B = self.config.angRateInit_B
        swInitAttitudeFrame = simParam.simOptions.swInitAttitudeFrame
        inertiaSc_B = spacecraft.massModel.inertiaSc_B

        if (swInitAttitudeFrame == "L"):
            # Initial attitude is initialized wrt nadir reference frame
            qLI = attKin.Quaternion(qLI_sca, qLI_vec)
            angRate_BL_B = angRateInit_B
            qBL = qInit
            # Initialize attitude and angular rates
            angRate_BI_B = angRate_BL_B + np.matmul(qBL.toDcm(), angRate_LI_L)
            qBI = attKin.multiplyQuat(qLI, qBL)
        elif (swInitAttitudeFrame == "J"):
            # Initial attitude is initialized wrt inertial reference frame
            angRate_BI_B = angRateInit_B
            qBI = qInit
        else:
            # Initial attitude is initialized wrt inertial reference frame by default
            angRate_BI_B = np.zeros(3)
            qBI = attKin.Quaternion()
        qBI = qBI.normalize()   

        # Compute initial angular momentum
        angMomSc_B = computeAngMom(angRate_BI_B, inertiaSc_B)

        # Update state
        self.state.qBI = qBI
        self.state.angRate_BI_B = angRate_BI_B

        # Update interface
        self.interface["qBI"] = qBI
        self.interface["angRate_BI_B"] = angRate_BI_B
        self.interface["angMomSc_B"] = angMomSc_B

    def run(self, simParam, spacecraft, modelsIf):
        # Retrieve inputs
        massModel = spacecraft.massModel
        inertiaSc_B = massModel.inertiaSc_B

        # Fecth models interface
        torqueTot_B = modelsIf["torqueTot_B"]

        # Process
        # 1. Propagate attitude state
        self.state.propagate(massModel, simParam, torqueTot_B)
        # 2. Compute spacecraft angular momentum
        # angMomRw_B = np.array([0, 0, 0]) # RW models to be implemented
        angMomSc_B = computeAngMom(self.state.angRate_BI_B, inertiaSc_B)
        # Update interface
        self.interface["qBI"] = self.state.qBI
        self.interface["angRate_BI_B"] = self.state.angRate_BI_B
        self.interface["angMomSc_B"] = angMomSc_B


# To be implemented as dataclass
class Config():
    def __init__(self):
        self.qInit = attKin.Quaternion()
        self.angRateInit_B = np.array([0, 0, 0])


class State():
    def __init__(self, qBI = attKin.Quaternion(), angRate_BI_B = np.array([0, 0, 0])):
        self.qBI = qBI
        self.angRate_BI_B = angRate_BI_B
            
    def propagate(self, massModel, simParam, torqueTot_B):
        self.qBI.propagateState(self.angRate_BI_B, simParam)
        self.angRate_BI_B = propagateAngRates(self.angRate_BI_B, torqueTot_B, massModel, simParam)
    

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
# Angular acceleration
def angRatesDot(angRates, externalTorque, massModel):   
    # Retrieve useful inputs
    scInertia    = massModel.inertiaSc_B
    scInertiaInv = massModel.inertiaScInv_B
    # Process
    angRatesDot  = np.matmul(scInertiaInv, externalTorque - np.cross(angRates, np.matmul(scInertia, angRates)))
    
    return angRatesDot

# Angular rates propagation
def propagateAngRates(angRatesPrev, externalTorquePrev, massModel, simParam):
    # Retrieve useful inputs
    xPrev = angRatesPrev
    Ts = simParam.Ts
    # Process
    # RK4 propagation
    k1 = angRatesDot(xPrev, externalTorquePrev, massModel)
    x2 = xPrev + Ts/2 * k1
    k2 = angRatesDot(x2, externalTorquePrev, massModel)
    x3 = xPrev + Ts/2 * k2
    k3 = angRatesDot(x3, externalTorquePrev, massModel)
    x4 = xPrev + Ts * k3
    k4 = angRatesDot(x4, externalTorquePrev, massModel)
    angRatesNext = xPrev + Ts/6 * (k1 + 2*k2 + 2*k3 + k4)
     
    return angRatesNext


# Angular momentum
# TBW: include RW angular momentum
def computeAngMom(angRate_BI_B, inertiaSc_B):
    # Process
    angMomSc_B = np.matmul(inertiaSc_B, angRate_BI_B)
    return angMomSc_B
    

# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------