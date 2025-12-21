
import numpy as np

import src.utils.paramTools as paramTools

import src.models.kin.attitudeKinematics as attKin

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# Guidance function
class Guidance():
    def __init__(self):
        # Parameters
        self.param = Param()
        # State 
        self.state = State()
        # Interface
        self.interface = {}
        self.interface.update({"eulerAng_RI": np.array([0, 0, 0])})
        self.interface.update({"eulerAng_RL": np.array([0, 0, 0])})
        self.interface.update({"qRI": attKin.Quaternion()})    
        self.interface.update({"angRate_RI_R": np.array([0, 0, 0])})

    # Build
    def build(self, patches):
        # Patch parameters
        self.param = paramTools.patchAttributes(self.param, patches["guidParamPatch"])
        # Set state
        self.state = State()

    # Run
    def run(self, fswModeMgtIf, fswEstIf):
        # Fetch interfaces
        qLI = fswEstIf["qLIest"]
        angRate_LI_L = fswEstIf["angRateEst_LI_L"]
        aocsGuidMode = fswModeMgtIf["aocsGuidMode"]

        # Compute guidance angular rates and euler angles depending on the current guidance mode
        # Guidance modes:
        # GUIDMODE_OFF (Off)
        # GUIDMODE_RATE_DAMPING (Rate damping)
        # GUIDMODE_ATT_NADIR (Nadir pointing)
        # GUIDMODE_SLEW (Slew) => TBW
        # GUIDMODE_ATT_INERT (Attitude inertial pointing)

        if (aocsGuidMode == "GUIDMODE_ATT_INERT"):
            # Inertial attitude guidance mode
            (angRate_RI_R, qRI) = self.attitudeInertialGuidance()
        elif (aocsGuidMode == "GUIDMODE_RATE_DAMPING"):
            # Rate damping guidance mode
            (angRate_RI_R, qRI) = self.rateDampingGuidance()
        elif (aocsGuidMode == "GUIDMODE_ATT_NADIR"):
            # Nadir guidance mode
            (angRate_RI_R, qRI) = self.nadirGuidance(angRate_LI_L, qLI)
        else:
            # OFF guidance mode or current mode is not defined
            aocsGuidMode = "GUIDMODE_OFF"
            (angRate_RI_R, qRI) = self.offGuidance()   
        
        qRI.normalize()
        
        # Compute guidance with respect to the nadir reference frame
        qRL = attKin.multiplyQuat(qLI.conjugate(), qRI)

        # Compute euler angles
        eulerAng_RI = qRI.toEuler() 
        eulerAng_RL = qRL.toEuler()

        # Update interface
        self.interface["eulerAng_RI"] = eulerAng_RI
        self.interface["eulerAng_RL"] = eulerAng_RL
        self.interface["qRI"] = qRI
        self.interface["angRate_RI_R"] = angRate_RI_R


    def rateDampingGuidance(self):
        angRate_RI_R = np.array([0, 0, 0])
        qRI = attKin.Quaternion()
        return (angRate_RI_R, qRI)


    def attitudeInertialGuidance(self):
        fswGuidanceParam = self.param
        angRate_RI_R = np.array([0, 0, 0])
        qRI = attKin.trans_EulerAngToQuat(fswGuidanceParam.GUIDMODE_ATT_INERT_eulerAngGuid_RI)
        return (angRate_RI_R, qRI)


    def offGuidance(self):
        angRate_RI_R = np.array([0, 0, 0])
        qRI = attKin.Quaternion()
        return (angRate_RI_R, qRI)


    def nadirGuidance(self, angRate_LI_L, qLI):
        fswGuidanceParam = self.param
        angRate_RI_R = angRate_LI_L
        qRI = qLI
        return (angRate_RI_R, qRI)



# Guidance function state
class State:
    def __init__(self):
        # [TBW] Dummy state
        self.dummy = 0


# Guidance function parameters
class Param:
    def __init__(self):
        self.GUIDMODE_ATT_INERT_eulerAngGuid_RI = np.array([0, 0, 0])*deg2rad
        self.orbitRate = 0 # [rad/s]        


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
# ...


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------
# ...

