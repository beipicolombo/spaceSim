# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np
import src.models.kin.attitudeKinematics as attitudeKinematics


# --------------------------------------------------
# CLASSES
# --------------------------------------------------

class FswControlParam:
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
def thrustControl(fswControlParam):
    torqueCtrl_B = np.array([0, 0, 0]) 
    forceCtrl_B = np.array([0, 0, 0])
    return forceCtrl_B

def rateControl(fswControlParam, angRateEst_BR_B):
    torqueCtrl_B = -np.matmul(fswControlParam.rateDampingKd, angRateEst_BR_B) 
    return torqueCtrl_B    

def attitudeControl(fswControlParam, angRateEst_BR_B, qEstBR):
    eulerAngEst_BR = qEstBR.toEuler()
    #torqueCtrl_B = -np.matmul(fswControlParam.attControlKd, angRateEst_BR_B) - np.matmul(fswControlParam.attControlKp, qEstBR.vec)
    torqueCtrl_B = -np.matmul(fswControlParam.attControlKd, angRateEst_BR_B) - np.matmul(fswControlParam.attControlKp, eulerAngEst_BR)
    return torqueCtrl_B

def offControl() :
    forceCtrl_B = np.array([0, 0, 0])
    torqueCtrl_B = np.array([0, 0, 0])
    return (forceCtrl_B, torqueCtrl_B)


def inertiaTrqCompensation(fswControlParam, angRateEst_BI_B):
    inertiaSc_B = fswControlParam.inertiaSc_B
    trq_B = - np.cross(angRateEst_BI_B, np.matmul(inertiaSc_B, angRateEst_BI_B))
    trq_B = np.array([0, 0, 0])
    return trq_B

def manualAttitudeControl(trqCmdManual_B):
    return trqCmdManual_B

def computeControl(fswControlParam, fswBus):
    # Initialize output bus
    fswControlBusOut = fswBus.subBuses["control"]

    # Retrieve useful inputs
    trqCmdManual_B = fswBus.subBuses["interfaces"].signals["trqCmdManual_B"].value

    angRate_RI_R = fswBus.subBuses["guidance"].signals["angRate_RI_R"].value
    qRI_sca = fswBus.subBuses["guidance"].signals["qRI_sca"].value
    qRI_vec = fswBus.subBuses["guidance"].signals["qRI_vec"].value

    angRateEst_BI_B =  fswBus.subBuses["estimation"].signals["angRateEst_BI_B"].value
    qBIest_sca =  fswBus.subBuses["estimation"].signals["qBIest_sca"].value
    qBIest_vec =  fswBus.subBuses["estimation"].signals["qBIest_vec"].value

    aocsCtrMode = fswBus.subBuses["modeMgt"].signals["aocsCtrMode"].value
    aocsCtrActMode = fswBus.subBuses["modeMgt"].signals["aocsCtrActMode"].value

    # Compute the spacecraft attitude and angular rates with respect to the guidance reference frame
    qBIest = attitudeKinematics.Quaternion(qBIest_sca, qBIest_vec)
    qRI = attitudeKinematics.Quaternion(qRI_sca, qRI_vec)
    qEstBR = attitudeKinematics.multiplyQuat(qRI.conjugate(), qBIest).normalize()

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
        (forceCtrl_B, torqueCtrl_B) = offControl()
    elif (aocsCtrMode == "CTRLMODE_RATE_DAMP_CTRL"):
        # Rate damping control mode
        torqueCtrl_B = rateControl(fswControlParam, angRateEst_BR_B)
        forceCtrl_B = np.array([0, 0, 0])
    elif (aocsCtrMode == "CTRLMODE_ATT_CTRL"):
        # Inertial attitude control mode
        torqueCtrl_B = attitudeControl(fswControlParam, angRateEst_BR_B, qEstBR)
        forceCtrl_B = np.array([0, 0, 0])
    elif (aocsCtrMode == "CTRLMOD_THRUST_CTRL"):
        torqueCtrl_B = attitudeControl(fswControlParam, angRateEst_BR_B, qEstBR)
        forceCtrl_B = np.array([0, 0, 0])
    elif (aocsCtrMode == "CTRLMOD_MANUAL"):
        torqueCtrl_B = manualAttitudeControl(trqCmdManual_B)
        forceCtrl_B = np.array([0, 0, 0])
    else:
        # OFF control mode or current mode is not defined
        (forceCtrl_B, torqueCtrl_B) = offControl()

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
        
    # Update output bus signals
    fswControlBusOut.signals["forceCtrl_B"].update(forceCtrl_B)
    fswControlBusOut.signals["torqueCtrl_B"].update(torqueCtrl_B)
    fswControlBusOut.signals["forceCtrlThr_B"].update(forceCtrlThr_B)
    fswControlBusOut.signals["torqueCtrlThr_B"].update(torqueCtrlThr_B)
    fswControlBusOut.signals["torqueCtrlRw_B"].update(torqueCtrlRw_B)
    fswControlBusOut.signals["angRateEst_BR_B"].update(angRateEst_BR_B)
    fswControlBusOut.signals["eulerAngEst_BR"].update(eulerAngEst_BR)
    fswControlBusOut.signals["rotAngEst_BR"].update(rotAngEst_BR)
    fswControlBusOut.signals["qEstBR_sca"].update(qEstBR.sca)
    fswControlBusOut.signals["qEstBR_vec"].update(qEstBR.vec)
        
    return (fswControlBusOut)

# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------