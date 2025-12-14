
import numpy as np
import src.models.kin.attitudeKinematics as attKin


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Control():
    def __init__(self):
        self.param = Param()
        self.state = 0
        # TBW: implement separate classes for different control algorithms
        # Interface
        self.interface = {}
        self.interface.update({"forceCtrl_B": np.array([0, 0, 0])})
        self.interface.update({"torqueCtrl_B": np.array([0, 0, 0])})
        self.interface.update({"forceCtrlThr_B": np.array([0, 0, 0])})
        self.interface.update({"torqueCtrlThr_B": np.array([0, 0, 0])})
        self.interface.update({"torqueCtrlRw_B": np.array([0, 0, 0])})
        self.interface.update({"angRateEst_BR_B": np.array([0, 0, 0])})
        self.interface.update({"eulerAngEst_BR": np.array([0, 0, 0])})
        self.interface.update({"rotAngEst_BR": np.array([0, 0, 0])})
        self.interface.update({"qEstBR": attKin.Quaternion()})


    def thrustControl(self):
        torqueCtrl_B = np.array([0, 0, 0]) 
        forceCtrl_B = np.array([0, 0, 0])
        return forceCtrl_B


    def rateControl(self, angRateEst_BR_B):
        fswControlParam = self.param
        torqueCtrl_B = -np.matmul(fswControlParam.rateDampingKd, angRateEst_BR_B) 
        return torqueCtrl_B    


    def attitudeControl(self, angRateEst_BR_B, qEstBR):
        fswControlParam = self.param
        eulerAngEst_BR = qEstBR.toEuler()
        #torqueCtrl_B = -np.matmul(fswControlParam.attControlKd, angRateEst_BR_B) - np.matmul(fswControlParam.attControlKp, qEstBR.vec)
        torqueCtrl_B = -np.matmul(fswControlParam.attControlKd, angRateEst_BR_B) - np.matmul(fswControlParam.attControlKp, eulerAngEst_BR)
        return torqueCtrl_B


    def offControl(self):
        forceCtrl_B = np.array([0, 0, 0])
        torqueCtrl_B = np.array([0, 0, 0])
        return (forceCtrl_B, torqueCtrl_B)


    def inertiaTrqCompensation(self, angRateEst_BI_B):
        fswControlParam = self.param
        inertiaSc_B = fswControlParam.inertiaSc_B
        trq_B = - np.cross(angRateEst_BI_B, np.matmul(inertiaSc_B, angRateEst_BI_B))
        trq_B = np.array([0, 0, 0])

        return trq_B

    def manualAttitudeControl(self, trqCmdManual_B):
        return trqCmdManual_B


    def run(self, fswModeMgtIf, fswEstIf, fswGuidIf):
        # Retrieve parameters
        fswControlParam = self.param

        # Fetch mode management interface
        aocsCtrMode = fswModeMgtIf["aocsCtrMode"]
        aocsCtrActMode = fswModeMgtIf["aocsCtrActMode"]
        # Fetch guidance interface
        angRate_RI_R = fswGuidIf["angRate_RI_R"]
        qRI = fswGuidIf["qRI"]
        # Fetch estimation interface
        angRateEst_BI_B =  fswEstIf["angRateEst_BI_B"]
        qBIest =  fswEstIf["qBIest"]

        # TBW Fetch manual inputs interface
        # trqCmdManual_B = fswBus.subBuses["interfaces"].signals["trqCmdManual_B"].value
        trqCmdManual_B = np.array([0, 0, 0])

        # Compute quaternion error
        qEstBR = attKin.multiplyQuat(qRI.conjugate(), qBIest).normalize()
        # Compute rate error
        angRateEst_BR_B = angRateEst_BI_B - np.matmul(qEstBR.toDcm(), angRate_RI_R)
        eulerAngEst_BR = qEstBR.toEuler()
        (rotDirEst_BR_B, rotAngEst_BR) = qEstBR.toVecRot()

        # Compute control toraque depending on the current control mode
        # Control modes:
        # CTRLMODE_OFF (Off)
        # CTRLMODE_ATT_CTRL (Attitude control)
        # CTRLMODE_RATE_DAMP_CTRL (Rate control)
        # CTRLMOD_THRUST_CTRL  (Thrust control)
        # CTRLMOD_MANUAL (Manual control)

        if (aocsCtrMode == "CTRLMODE_OFF"):
            # OFF control mode
            (forceCtrl_B, torqueCtrl_B) = self.offControl()
        elif (aocsCtrMode == "CTRLMODE_RATE_DAMP_CTRL"):
            # Rate damping control mode
            torqueCtrl_B = self.rateControl(angRateEst_BR_B)
            forceCtrl_B = np.array([0, 0, 0])
        elif (aocsCtrMode == "CTRLMODE_ATT_CTRL"):
            # Inertial attitude control mode
            torqueCtrl_B = self.attitudeControl(angRateEst_BR_B, qEstBR)
            forceCtrl_B = np.array([0, 0, 0])
        elif (aocsCtrMode == "CTRLMOD_THRUST_CTRL"):
            torqueCtrl_B = self.attitudeControl(angRateEst_BR_B, qEstBR)
            forceCtrl_B = np.array([0, 0, 0])
        elif (aocsCtrMode == "CTRLMOD_MANUAL"):
            torqueCtrl_B = self.manualAttitudeControl(trqCmdManual_B)
            forceCtrl_B = np.array([0, 0, 0])
        else:
            # OFF control mode or current mode is not defined
            (forceCtrl_B, torqueCtrl_B) = self.offControl()

        # Apply feed-forward inertia torque compensation
        if fswControlParam.swInertiaTrqCompensation:
            torqueCtrl_B = torqueCtrl_B + inertiaTrqCompensation(fswControlParam, angRateEst_BI_B)        

        
        # Switch between THR and RW control torques depending on the actuator mode
        # Actuator modes:
        # NONE (None)
        # THR (Thrusters)
        # RW (Reaction wheels) => TBW
        # RW_OFFLOADING (Reaction wheels offloading) => TBW

        if (aocsCtrActMode == "THR"):
            # THR only
            torqueCtrlThr_B = torqueCtrl_B
            forceCtrlThr_B = forceCtrl_B
            torqueCtrlRw_B = np.array([0, 0, 0])
        elif (aocsCtrActMode == "RW"):
            # RW only
            torqueCtrlThr_B = np.array([0, 0, 0])
            forceCtrlThr_B = np.array([0, 0, 0])
            torqueCtrlRw_B = torqueCtrl_B
        else :
            # RW_OFFLOADING => TBW
            torqueCtrlThr_B = np.array([0, 0, 0])
            forceCtrlThr_B = np.array([0, 0, 0])
            torqueCtrlRw_B = np.array([0, 0, 0])
            
        # Update interface
        self.interface["forceCtrl_B"] = forceCtrl_B
        self.interface["torqueCtrl_B"] = torqueCtrl_B
        self.interface["forceCtrlThr_B"] = forceCtrlThr_B
        self.interface["torqueCtrlThr_B"] = torqueCtrlThr_B
        self.interface["torqueCtrlRw_B"] = torqueCtrlRw_B
        self.interface["angRateEst_BR_B"] = angRateEst_BR_B
        self.interface["eulerAngEst_BR"] = eulerAngEst_BR
        self.interface["rotAngEst_BR"] = rotAngEst_BR
        self.interface["qEstBR"] = qEstBR
            

class Param:
    # To be moved as higher level class
    def __init__(self):
        self.rateDampingKd = 10.0*np.eye(3)
        self.attControlKp = 0.1*np.eye(3)
        self.attControlKd = 10*np.eye(3)
        self.inertiaSc_B = np.identity(3) # Inertia at the center of mass, in body frame [kg*m^2]
        self.swInertiaTrqCompensation = False


    def getAttCtrlTuningFrom_overshootAndTimeResp(self, overshoot, timeResp):
        # Get damping
        damp = np.sqrt(np.log(overshoot)**2/(np.pi**2 + np.log(overshoot)**2))
        # Get natural pulse
        omega_n = 2/(timeResp * damp)

        # Get controller gains
        Kp = omega_n**2 * self.inertiaSc_B
        Kd = 2*damp*omega_n * self.inertiaSc_B
        
        print("damping: %.4f [-]" % (damp))
        print("natural frequency: %.4f [rad/s]" % (omega_n))

        self.attControlKd = Kd
        self.attControlKp = Kp


    def getAttCtrlTuningFrom_dampingAndNaturalFrequency(self, damp, omega_n):
        # Get controller gains
        Kp = omega_n**2 * self.inertiaSc_B
        Kd = 2*damp*omega_n * self.inertiaSc_B

        self.attControlKd = Kd
        self.attControlKp = Kp


    def getRateCtrlTuningFrom_timeResp(self, timeResp):
        # 2% time response formula: Tr(2%) = 4 x tau, with tau = 1/omega (1st order system cutoff frequency)
        # Get controller gain
        Kd = self.inertiaSc_B / timeResp

        self.rateDampingKd = Kd


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------