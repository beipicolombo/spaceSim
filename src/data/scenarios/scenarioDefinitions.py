
import numpy as np

import ephem
import src.attitudeKinematics as attitudeKinematics


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


def scenarioDefinition_template():
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



def scenarioDefinition_testDevelopment():
    print("Patches...")
    
    # Initialize output
    patches = {}
    
    # Simulation parameters
    simParamPatch = {}
    simParamPatch["caseName"] = "testDevelopment"
    simParamPatch["dateTimeStart"] = ephem.Date("2024/3/9 5:10:10")
    simParamPatch["Ts"] = 0.5 # [s]
    simParamPatch["Tend"] = 3*60 # [s]
    patches.update({"simParamPatch": simParamPatch})
    
    runOptionsPatch = {}
    runOptionsPatch["isPlot"] = True
    runOptionsPatch["swVisualPyLabelsAttitude"] = True
    runOptionsPatch["visualPyRate"] = 300
    patches.update({"runOptionsPatch": runOptionsPatch})
    
    simOptionsPatch = {}
    simOptionsPatch["isGGTorqueEnabled"] = False
    simOptionsPatch["isCtrlTorqueEnabled"] = True
    simOptionsPatch["swInitAttitudeFrame"] = "J"
    simOptionsPatch["massModelName"] = "SC_100kg_rect"
    patches.update({"simOptionsPatch": simOptionsPatch})
    
    # Initial attitude
    initDynStatePatch = {}
    initDynStatePatch["qInitVec"] = attitudeKinematics.trans_EulerAngToQuat(np.array([0, 0, 0]) * deg2rad).toVec()
    initDynStatePatch["angRateInit_B"] = np.array([2.0, 0.5, 0.9])*deg2rad
    patches.update({"initDynStatePatch": initDynStatePatch})
    
    # Initial orbit
    orbitInitParamPatch = {}
    orbitInitParamPatch["perAltitudeInit"] = 500 * 1e3 # [m]
    orbitInitParamPatch["ecc"] = 0.01
    orbitInitParamPatch["inc"] = 10 * deg2rad # [rad]
    orbitInitParamPatch["raan"] = 10 * deg2rad # [rad]
    orbitInitParamPatch["argPer"] = 90 * deg2rad # [rad]
    orbitInitParamPatch["ta"] = 0 * deg2rad # [rad]
    patches.update({"orbitInitParamPatch": orbitInitParamPatch})
    
    # Mode management
    modeMgtParamPatch = {}
    modeMgtParamPatch["aocsModeInit"] = "SAFE"
    modeMgtParamPatch["aocsOffModeMinDur"] = 10
    modeMgtParamPatch["aocsSafeModeAngRateThdDur"] = 10*60
    modeMgtParamPatch["aocsSafeModeAngRateThd"] = 0.02*deg2rad
    modeMgtParamPatch["isAutoSafeToNomPtngModeAllwd"] = True
    modeMgtParamPatch["isAutoNomPtngToNomEqAllwd"] = True
    patches.update({"modeMgtParamPatch": modeMgtParamPatch})
    
    # Guidance
    guidParamPatch = {}
    guidParamPatch["GUIDMODE_ATT_INERT_eulerAngGuid_RI"] = np.array([0, 45, 45]) * deg2rad
    patches.update({"guidParamPatch": guidParamPatch})
    
    # Control
    ctrParamPatch = {}
    ctrParamPatch["swInertiaTrqCompensation"] = False
    patches.update({"ctrParamPatch": ctrParamPatch})

    return patches
    
    