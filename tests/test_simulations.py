import pytest
import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
import src.utils.nonRegression as nonRegression 

# Each scenario defines input + expected output
scenarios = [
    {
        "name": "testDevelopment",
        "scenarioPatchFcnHdl": scenarioDefinitions.scenarioDefinition_testDevelopment,
    }
]


@pytest.mark.parametrize("scenario", scenarios)
def test_simulations(scenario):
    nonRegressionOutput = nonRegression.run(scenario["scenarioPatchFcnHdl"])
    assert nonRegressionOutput["isAllOk"] == True, f"Scenario failed: {scenario['name']}"

