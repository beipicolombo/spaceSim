

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from src.data.scenarios.scenarioDefinitions import AVAILABLE_SCENARIOS
from src.utils.runSimu import runSimu


# Run the simulation
scenarioPatchFcnHdl = AVAILABLE_SCENARIOS["testDevelopment"]
out = runSimu(scenarioPatchFcnHdl)

print(out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.timeVec.shape)
print(out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.rad2deg().dataVec[:,0].shape)
print(out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.rad2deg().dataVec[:,0].shape)