
import numpy as np

import ephem
import src.attitudeKinematics as attitudeKinematics
import src.interfaces.eventsAndTmTc as eventsAndTmTc


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


def scenarioDefinition_template(isReference = False):
    print("Patches...")
    
    # Initialize output
    patches = {}
    
    # Simulation parameters
    simParamPatch = {}
    simParamPatch["caseName"] = "template"
    patches.update({"simParamPatch": simParamPatch})
    
    runOptionsPatch = {}
    patches.update({"runOptionsPatch": runOptionsPatch})
    
    simOptionsPatch = {}
    patches.update({"simOptionsPatch": simOptionsPatch})
    
    # Initial attitude
    initDynStatePatch = {}
    patches.update({"initDynStatePatch": initDynStatePatch})
    
    # Initial orbit
    orbitInitParamPatch = {}
    patches.update({"orbitInitParamPatch": orbitInitParamPatch})
    
    # Mode management
    modeMgtParamPatch = {}
    patches.update({"modeMgtParamPatch": modeMgtParamPatch})
    
    # Guidance
    guidParamPatch = {}
    patches.update({"guidParamPatch": guidParamPatch})
    
    # Control
    ctrParamPatch = {}
    patches.update({"ctrParamPatch": ctrParamPatch})

    return patches


def scenarioDefinition_manualModeDev(isReference = False):
    print("Patches...")
    
    # Initialize output
    patches = {}
    
    # Simulation parameters
    simParamPatch = {}
    simParamPatch["caseName"] = "manualModeDev"
    simParamPatch["dateTimeStart"] = ephem.Date("2024/3/9 5:10:10")
    simParamPatch["Ts"] = 0.5 # [s]
    simParamPatch["Tend"] = 5*60 # [s]
    patches.update({"simParamPatch": simParamPatch})
    
    runOptionsPatch = {}
    signalPathsToLog = []
    signalPathsToLog.append("modelsBus/dynamics/attitude/angRate_BI_B")
    signalPathsToLog.append("modelsBus/dynamics/attitude/eulerAng_BI")
    signalPathsToLog.append("modelsBus/dynamics/attitude/torqueTot_B")
    signalPathsToLog.append("modelsBus/dynamics/attitude/angMomSc_B")
    signalPathsToLog.append("modelsBus/dynamics/posVel/pos_I")
    signalPathsToLog.append("modelsBus/dynamics/orbitElem/argPer")
    signalPathsToLog.append("modelsBus/dynamics/orbitElem/ta")
    signalPathsToLog.append("modelsBus/performance/eulerAng_BL")
    signalPathsToLog.append("modelsBus/performance/angRate_BL_B")
    signalPathsToLog.append("modelsBus/environment/torqueExt_B")
    signalPathsToLog.append("modelsBus/environment/eulerAng_LI")
    signalPathsToLog.append("modelsBus/environment/qLI_sca")
    signalPathsToLog.append("modelsBus/environment/qLI_vec")
    signalPathsToLog.append("modelsBus/environment/angRate_LI_L")
    signalPathsToLog.append("modelsBus/actuators/torqueThr_B")
    signalPathsToLog.append("fswBus/guidance/angRate_RI_R")
    signalPathsToLog.append("fswBus/guidance/eulerAng_RI")
    signalPathsToLog.append("fswBus/guidance/eulerAng_RL")
    signalPathsToLog.append("fswBus/modeMgt/aocsModeElapsedTime")
    signalPathsToExport = []
    signalPathsToExport.append("fswBus/estimation/posEst_J")
    signalPathsToExport.append("fswBus/estimation/velEst_J")
    signalPathsToExport.append("fswBus/estimation/angRateEst_LI_L")
    signalPathsToExport.append("fswBus/estimation/angRateEst_LI_L")
    runOptionsPatch["signalPathsToLog"] = signalPathsToLog
    runOptionsPatch["signalPathsToExport"] = signalPathsToExport
    runOptionsPatch["isReference"] = False
    patches.update({"runOptionsPatch": runOptionsPatch})
    
    simOptionsPatch = {}
    simOptionsPatch["isGGTorqueEnabled"] = False
    simOptionsPatch["massModelName"] = "SC_100kg_rect"
    patches.update({"simOptionsPatch": simOptionsPatch})
    
    # Initial attitude
    initDynStatePatch = {}
    initDynStatePatch["qInitVec"] = attitudeKinematics.trans_EulerAngToQuat(np.array([0, 0, 0]) * deg2rad).toVec()
    patches.update({"initDynStatePatch": initDynStatePatch})
    
    # Initial orbit
    orbitInitParamPatch = {}
    patches.update({"orbitInitParamPatch": orbitInitParamPatch})
    
    # Interface
    ifKeyboardParamPatch = {}
    ifKeyboardParamPatch["isKeyboardUsed"] = True
    patches.update({"ifKeyboardParamPatch": ifKeyboardParamPatch})

    # Mode management
    modeMgtParamPatch = {}
    modeMgtParamPatch["aocsModeInit"] = "MANUAL"
    patches.update({"modeMgtParamPatch": modeMgtParamPatch})
    
    # Guidance
    guidParamPatch = {}
    patches.update({"guidParamPatch": guidParamPatch})
    
    # Control
    ctrParamPatch = {}
    patches.update({"ctrParamPatch": ctrParamPatch})

    return patches


def scenarioDefinition_nominalScenario(isReference = False):
    print("Patches...")
    
    # Initialize output
    patches = {}
    
    # Simulation parameters
    simParamPatch = {}
    simParamPatch["caseName"] = "testDevelopment"
    simParamPatch["dateTimeStart"] = ephem.Date("2024/3/9 5:10:10")
    simParamPatch["Ts"] = 0.5 # [s]
    simParamPatch["Tend"] = 2*90*60 # [s]
    patches.update({"simParamPatch": simParamPatch})
    
    runOptionsPatch = {}
    signalPathsToLog = []
    signalPathsToLog.append("modelsBus/dynamics/attitude/angRate_BI_B")
    signalPathsToLog.append("modelsBus/dynamics/attitude/eulerAng_BI")
    signalPathsToLog.append("modelsBus/dynamics/attitude/torqueTot_B")
    signalPathsToLog.append("modelsBus/dynamics/attitude/angMomSc_B")
    signalPathsToLog.append("modelsBus/dynamics/posVel/pos_I")
    signalPathsToLog.append("modelsBus/dynamics/orbitElem/argPer")
    signalPathsToLog.append("modelsBus/dynamics/orbitElem/ta")
    signalPathsToLog.append("modelsBus/performance/eulerAng_BL")
    signalPathsToLog.append("modelsBus/performance/angRate_BL_B")
    signalPathsToLog.append("modelsBus/environment/torqueExt_B")
    signalPathsToLog.append("modelsBus/environment/eulerAng_LI")
    signalPathsToLog.append("modelsBus/environment/qLI_sca")
    signalPathsToLog.append("modelsBus/environment/qLI_vec")
    signalPathsToLog.append("modelsBus/environment/angRate_LI_L")
    signalPathsToLog.append("modelsBus/actuators/torqueThr_B")
    signalPathsToLog.append("fswBus/guidance/angRate_RI_R")
    signalPathsToLog.append("fswBus/guidance/eulerAng_RI")
    signalPathsToLog.append("fswBus/guidance/eulerAng_RL")
    signalPathsToLog.append("fswBus/modeMgt/aocsModeElapsedTime")
    signalPathsToExport = []
    signalPathsToExport.append("fswBus/estimation/posEst_J")
    signalPathsToExport.append("fswBus/estimation/velEst_J")
    signalPathsToExport.append("fswBus/estimation/angRateEst_LI_L")
    signalPathsToExport.append("fswBus/estimation/angRateEst_LI_L")  
    runOptionsPatch["signalPathsToLog"] = signalPathsToLog
    runOptionsPatch["signalPathsToExport"] = signalPathsToExport
    # runOptionsPatch["isReference"] = True
    patches.update({"runOptionsPatch": runOptionsPatch})
    
    simOptionsPatch = {}
    simOptionsPatch["isGGTorqueEnabled"] = False
    simOptionsPatch["massModelName"] = "SC_100kg_rect"
    patches.update({"simOptionsPatch": simOptionsPatch})
    
    # Initial attitude
    initDynStatePatch = {}
    initDynStatePatch["qInitVec"] = attitudeKinematics.trans_EulerAngToQuat(np.array([0, 0, 0]) * deg2rad).toVec()
    initDynStatePatch["angRateInit_B"] = np.array([2.0, 0.5, 0.9])*deg2rad
    patches.update({"initDynStatePatch": initDynStatePatch})
    
    # Initial orbit
    orbitInitParamPatch = {}
    patches.update({"orbitInitParamPatch": orbitInitParamPatch})

    # Interface
    ifKeyboardParamPatch = {}
    patches.update({"ifKeyboardParamPatch": ifKeyboardParamPatch})
    
    # Mode management
    modeMgtParamPatch = {}
    modeMgtParamPatch["aocsModeInit"] = "SAFE"
    modeMgtParamPatch["isAutoSafeToNomPtngModeAllwd"] = True
    modeMgtParamPatch["isAutoNomPtngToNomEqAllwd"] = True
    patches.update({"modeMgtParamPatch": modeMgtParamPatch})
    
    # Guidance
    guidParamPatch = {}
    guidParamPatch["GUIDMODE_ATT_INERT_eulerAngGuid_RI"] = np.array([0, 45, 45]) * deg2rad
    patches.update({"guidParamPatch": guidParamPatch})
    
    # Control
    ctrParamPatch = {}
    patches.update({"ctrParamPatch": ctrParamPatch})

    return patches


def scenarioDefinition_testDevelopment(isReference = False):
    print("Patches...")
    
    # Initialize output
    patches = {}
    
    # Simulation parameters
    simParamPatch = {}
    simParamPatch["caseName"] = "testDevelopment"
    simParamPatch["dateTimeStart"] = ephem.Date("2024/3/9 5:10:10")
    simParamPatch["Ts"] = 0.5 # [s]
    simParamPatch["Tend"] = 2*90*60 # [s]
    tcTimeline = []
    tcTimeline.append(eventsAndTmTc.Tc(name = "TC_AOCS_MODE_SWITCH_SAFE_TO_NOM_PTNG", id = 1, time = 10))
    tcTimeline.append(eventsAndTmTc.Tc(name = "TC_AOCS_MODE_SWITCH_SAFE_TO_NOM_PTNG", id = 1, time = 20))
    tcTimeline.append(eventsAndTmTc.Tc(name = "TC_AOCS_MODE_SWITCH_SAFE_TO_NOM_PTNG", id = 1, time = 30))
    tcTimeline.append(eventsAndTmTc.Tc(name = "TC_AOCS_MODE_SWITCH_SAFE_TO_NOM_PTNG", id = 1, time = 90*60))
    simParamPatch["tcTimeline"] = tcTimeline
    patches.update({"simParamPatch": simParamPatch})
    
    runOptionsPatch = {}
    signalPathsToLog = []
    signalPathsToExport = []
    runOptionsPatch["signalPathsToLog"] = signalPathsToLog
    runOptionsPatch["signalPathsToExport"] = signalPathsToExport
    runOptionsPatch["swExportData"] = False
    patches.update({"runOptionsPatch": runOptionsPatch})
    
    simOptionsPatch = {}
    simOptionsPatch["isGGTorqueEnabled"] = False
    simOptionsPatch["massModelName"] = "SC_100kg_rect"
    patches.update({"simOptionsPatch": simOptionsPatch})
    
    # Initial attitude
    initDynStatePatch = {}
    initDynStatePatch["qInitVec"] = attitudeKinematics.trans_EulerAngToQuat(np.array([0, 0, 0]) * deg2rad).toVec()
    initDynStatePatch["angRateInit_B"] = np.array([2.0, 0.5, 0.9])*deg2rad
    patches.update({"initDynStatePatch": initDynStatePatch})
    
    # Initial orbit
    orbitInitParamPatch = {}
    patches.update({"orbitInitParamPatch": orbitInitParamPatch})

    # Interface
    ifKeyboardParamPatch = {}
    patches.update({"ifKeyboardParamPatch": ifKeyboardParamPatch})
    
    # Mode management
    modeMgtParamPatch = {}
    modeMgtParamPatch["aocsModeInit"] = "SAFE"
    patches.update({"modeMgtParamPatch": modeMgtParamPatch})
    
    # Guidance
    guidParamPatch = {}
    guidParamPatch["GUIDMODE_ATT_INERT_eulerAngGuid_RI"] = np.array([0, 45, 45]) * deg2rad
    patches.update({"guidParamPatch": guidParamPatch})
    
    # Control
    ctrParamPatch = {}
    patches.update({"ctrParamPatch": ctrParamPatch})

    return patches
    
    
