import pytest
import numpy as np
import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
import src.utils.nonRegression as nonRegression
import src.models.kin.attitudeKinematics as attKin


# Each scenario defines input + expected output
quaternionMethodsToTest = [
    {"name": "validate_norm",},
    {"name": "validate_isIdentity",},
    {"name": "validate_normalize",},
    {"name": "validate_toVec"},
    {"name": "validate_toEuler"},
    {"name": "validate_toDcm"},
    {"name": "validate_toVecRot"},
    {"name": "validate_propagateState"},
]

kinematicsFunctionsToTest = [
    {"handle": attKin.validate_multiplyQuat},
    {"handle": attKin.validate_applyRotation},
    {"handle": attKin.validate_trans_VecRotToQuat},
    {"handle": attKin.validate_trans_VecToQuat},
    {"handle": attKin.validate_trans_VecToCrossMat},
    {"handle": attKin.validate_trans_AngVecToQuat},
    {"handle": attKin.validate_trans_EulerAngToQuat},
    {"handle": attKin.validate_trans_DcmToEulerAng},
    {"handle": attKin.validate_trans_DcmToQuat},
    {"handle": attKin.validate_trans_EulerAngToDcm},
    {"handle": attKin.validate_Rx},
    {"handle": attKin.validate_Ry},
    {"handle": attKin.validate_Rz},
]


@pytest.mark.parametrize("functionToTest", quaternionMethodsToTest)
def test_quatMethods(functionToTest):
    unitTestOutput = getattr(attKin.Quaternion(), functionToTest["name"])()
    assert unitTestOutput["isOk"] == True, f"Function failed: {functionToTest['name']}"

@pytest.mark.parametrize("functionToTest", kinematicsFunctionsToTest)
def test_kinFunctions(functionToTest):
    unitTestOutput = functionToTest["handle"]()
    assert unitTestOutput["isOk"] == True, f"Function failed: {functionToTest['handle'].__name__}"
