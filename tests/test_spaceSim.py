# test_validators.py
import pytest
from src.runSimuNew import runSimu
import src.data.scenarios.scenarioDefinitions as scenarioDefinitions


# Each scenario defines input + expected output
scenarios = [
    {
        "name": "DefaultSimulatorRun",
        "expected": {"isRunOk": True},
    }
]


# scenarioPatchFcnHdl = scenarioDefinitions.scenarioDefinition_testDevelopment
# (dictDataExport, simParam) = runSimu(scenarioPatchFcnHdl)

@pytest.mark.parametrize("scenario", scenarios)
def test_runScenario(scenario):
    scenarioPatchFcnHdl = scenarioDefinitions.scenarioDefinition_testDevelopment
    (dictDataExport, simParam) = runSimu(scenarioPatchFcnHdl)
    # Assertion to be implemented
    assert scenario["expected"]== {"isRunOk": True}, f"Scenario failed: {scenario['name']}"

