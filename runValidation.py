

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from src.utils.runSimu import runSimu


# Run the simulation
scenarioPatchFcnHdl = scenarioDefinitions.scenarioDefinition_testDevelopment
(dictDataExport, simParam) = runSimu(scenarioPatchFcnHdl)







