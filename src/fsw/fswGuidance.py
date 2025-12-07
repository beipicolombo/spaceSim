
import numpy as np
import src.models.kin.attitudeKinematics as attitudeKinematics

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180

# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Guidance():
    def __init__(self):
        self.param = Param()


    def rateDampingGuidance(self):
        angRate_RI_R = np.array([0, 0, 0])
        qRI = attitudeKinematics.Quaternion()
        return (angRate_RI_R, qRI)


    def attitudeInertialGuidance(self):
        fswGuidanceParam = self.param
        angRate_RI_R = np.array([0, 0, 0])
        qRI = attitudeKinematics.trans_EulerAngToQuat(fswGuidanceParam.GUIDMODE_ATT_INERT_eulerAngGuid_RI)
        return (angRate_RI_R, qRI)


    def offGuidance(self):
        angRate_RI_R = np.array([0, 0, 0])
        qRI = attitudeKinematics.Quaternion()
        return (angRate_RI_R, qRI)


    def nadirGuidance(self, angRate_LI_L, qLI):
        fswGuidanceParam = self.param
        angRate_RI_R = angRate_LI_L
        qRI = qLI
        return (angRate_RI_R, qRI)


    def run(self, fswBusIn):
        # Initialize output bus
        fswGuidBusOut = fswBusIn.subBuses["guidance"]

        # Retrieve signals / parameters
        aocsGuidMode = fswBusIn.subBuses["modeMgt"].signals["aocsGuidMode"].value
        angRate_LI_L = fswBusIn.subBuses["estimation"].signals["angRateEst_LI_L"].value
        qLI_sca = fswBusIn.subBuses["estimation"].signals["qLIest_sca"].value
        qLI_vec = fswBusIn.subBuses["estimation"].signals["qLIest_vec"].value

        # Reconstruct the LVLH frame quaternion
        qLI = attitudeKinematics.Quaternion(qLI_sca, qLI_vec)

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
        qRL = attitudeKinematics.multiplyQuat(qLI.conjugate(), qRI)

        # Compute euler angles
        eulerAng_RI = qRI.toEuler() 
        eulerAng_RL = qRL.toEuler()

        fswGuidBusOut.signals["eulerAng_RI"].update(eulerAng_RI)
        fswGuidBusOut.signals["eulerAng_RL"].update(eulerAng_RL)
        fswGuidBusOut.signals["qRI_sca"].update(qRI.sca)
        fswGuidBusOut.signals["qRI_vec"].update(qRI.vec)
        fswGuidBusOut.signals["angRate_RI_R"].update(angRate_RI_R)

        return fswGuidBusOut


class Param:
    # To be moved as higher level class
    def __init__(self):
        self.GUIDMODE_ATT_INERT_eulerAngGuid_RI = np.array([0, 0, 0])*deg2rad
        self.orbitRate = 0 # [rad/s]        


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------

