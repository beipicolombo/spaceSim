

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from src.data.scenarios.scenarioDefinitions import AVAILABLE_SCENARIOS
from src.utils.runSimu import runSimu


# Run the simulation
scenarioPatchFcnHdl = AVAILABLE_SCENARIOS["testDevelopment"]
out = runSimu(scenarioPatchFcnHdl)

print(out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.timeVec.shape)
# print(out["fswBus"].subBuses["modeMgt"].signals["aocsMode"].timeseries)
