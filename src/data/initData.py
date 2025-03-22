# -*- coding: utf-8 -*-
"""
Created on Thu Jan  2 11:38:17 2025

@author: Xela

"""


import src.utils.dataStructures as dataStructures

def initializeBusesAndSignals(simParam):
    modelsBus = dataStructures.DataBus("modelsBus")
    modelsBus.addSubBus("dynamics")
    modelsBus.subBuses["dynamics"].addSubBus("attitude")
    modelsBus.subBuses["dynamics"].subBuses["attitude"].addSignal("qBI_sca", 1, unit = "-")
    modelsBus.subBuses["dynamics"].subBuses["attitude"].addSignal("qBI_vec", 3, unit = "-")
    modelsBus.subBuses["dynamics"].subBuses["attitude"].addSignal("angRate_BI_B", 3, unit = "rad/s", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["dynamics"].subBuses["attitude"].addSignal("eulerAng_BI", 3, unit = "rad", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["dynamics"].subBuses["attitude"].addSignal("torqueTot_B", 3, unit = "N*m", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["dynamics"].subBuses["attitude"].addSignal("angMomSc_B", 3, unit = "N*m*s", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["dynamics"].addSubBus("posVel")
    modelsBus.subBuses["dynamics"].subBuses["posVel"].addSignal("pos_I", 3, unit = "m", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["dynamics"].subBuses["posVel"].addSignal("vel_I", 3, unit = "m")
    modelsBus.subBuses["dynamics"].addSubBus("orbitElem")
    modelsBus.subBuses["dynamics"].subBuses["orbitElem"].addSignal("sma", 3, unit = "m")
    modelsBus.subBuses["dynamics"].subBuses["orbitElem"].addSignal("ecc", 3)
    modelsBus.subBuses["dynamics"].subBuses["orbitElem"].addSignal("inc", 1, unit = "rad")
    modelsBus.subBuses["dynamics"].subBuses["orbitElem"].addSignal("raan", 1, unit = "rad")
    modelsBus.subBuses["dynamics"].subBuses["orbitElem"].addSignal("argPer", 1, unit = "rad", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["dynamics"].subBuses["orbitElem"].addSignal("ta", 1, unit = "rad", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["dynamics"].addSubBus("frames")
    modelsBus.addSubBus("performance")
    modelsBus.subBuses["performance"].addSignal("qBL_sca", 1, unit = "-")
    modelsBus.subBuses["performance"].addSignal("qBL_vec", 3, unit = "-")
    modelsBus.subBuses["performance"].addSignal("eulerAng_BL", 3, unit = "rad", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["performance"].addSignal("angRate_BL_B", 3, unit = "rad/s", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.addSubBus("environment")
    modelsBus.subBuses["environment"].addSignal("torqueGG_B", 3, unit = "N*m")
    modelsBus.subBuses["environment"].addSignal("torqueAero_B", 3, unit = "N*m")
    modelsBus.subBuses["environment"].addSignal("torqueExt_B", 3, unit = "N*m", isLogged = True, timeVec = simParam.timeVec)    
    modelsBus.subBuses["environment"].addSignal("eulerAng_LI", 3, unit = "rad", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["environment"].addSignal("qLI_sca", 1, unit = "-", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["environment"].addSignal("qLI_vec", 3, unit = "-", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.subBuses["environment"].addSignal("angRate_LI_L", 3, unit = "rad/s", isLogged = True, timeVec = simParam.timeVec)
    modelsBus.addSubBus("actuators")
    modelsBus.subBuses["actuators"].addSignal("torqueRw_B", 3, unit = "N*m")    
    modelsBus.subBuses["actuators"].addSignal("torqueThr_B", 3, unit = "N*m", isLogged = True, timeVec = simParam.timeVec)    
    modelsBus.subBuses["actuators"].addSignal("torqueAct_B", 3, unit = "N*m")
    modelsBus.addSubBus("sensors")
    modelsBus.subBuses["sensors"].addSignal("angRateMeas_BI_B", 3, unit = "rad/s", isLogged = True, timeVec = simParam.timeVec)    
    modelsBus.subBuses["sensors"].addSignal("eulerAngMeas_BI", 3, unit = "rad", isLogged = True, timeVec = simParam.timeVec)    
    modelsBus.subBuses["sensors"].addSignal("qBImeas_sca", 1, unit = "-")    
    modelsBus.subBuses["sensors"].addSignal("qBImeas_vec", 3, unit = "-")    
    fswBus = dataStructures.DataBus("fswBus")
    fswBus.addSubBus("guidance")
    fswBus.subBuses["guidance"].addSignal("angRate_RI_R", 3, unit = "rad/s", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["guidance"].addSignal("eulerAng_RI", 3, unit = "rad", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["guidance"].addSignal("eulerAng_RL", 3, unit = "rad", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["guidance"].addSignal("qRI_sca", 1, unit = "-")
    fswBus.subBuses["guidance"].addSignal("qRI_vec", 3, unit = "-")
    fswBus.addSubBus("control")
    fswBus.subBuses["control"].addSignal("qEstBR_sca", 1, unit = "-", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["control"].addSignal("qEstBR_vec", 3, unit = "-", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["control"].addSignal("rotAngEst_BR", 1, unit = "rad", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["control"].addSignal("eulerAngEst_BR", 3, unit = "rad", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["control"].addSignal("angRateEst_BR_B", 3, unit = "rad/s", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["control"].addSignal("forceCtrl_B", 3, unit = "N", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["control"].addSignal("torqueCtrl_B", 3, unit = "N*m", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["control"].addSignal("torqueCtrlRw_B", 3, unit = "N*m", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["control"].addSignal("forceCtrlThr_B", 3, unit = "N", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["control"].addSignal("torqueCtrlThr_B", 3, unit = "N*m", isLogged = True, timeVec = simParam.timeVec)
    fswBus.addSubBus("estimation")
    fswBus.subBuses["estimation"].addSignal("angRateEst_BI_B", 3, unit = "rad/s")
    fswBus.subBuses["estimation"].addSignal("eulerAngEst_BI", 3, unit = "rad")
    fswBus.subBuses["estimation"].addSignal("qBIest_sca", 1, unit = "-")
    fswBus.subBuses["estimation"].addSignal("qBIest_vec", 3, unit = "-")
    fswBus.addSubBus("command")
    fswBus.subBuses["command"].addSignal("torqueCmdThr_B", 3, unit = "N*m", isLogged = True, timeVec = simParam.timeVec)
    fswBus.subBuses["command"].addSignal("torqueCmdRw_B", 3, unit = "N*m", isLogged = True, timeVec = simParam.timeVec)
    fswBus.addSubBus("modeMgt")
    fswBus.subBuses["modeMgt"].addSignal("aocsMode", 1, unit = "-")
    fswBus.subBuses["modeMgt"].addSignal("aocsGuidMode", 1, unit = "-")
    fswBus.subBuses["modeMgt"].addSignal("aocsCtrMode", 1, unit = "-")
    fswBus.subBuses["modeMgt"].addSignal("aocsCtrActMode", 1, unit = "-")
    fswBus.subBuses["modeMgt"].addSignal("aocsModeElapsedTime", 1, unit = "s", isLogged = True, timeVec = simParam.timeVec)
    
    return (modelsBus, fswBus)