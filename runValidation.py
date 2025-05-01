

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from runSimu import *

scenarioPatchFcnHdl = scenarioDefinitions.scenarioDefinition_testDevelopment
(modelsBus, fswBus, simBus, simParam) = runSimu(scenarioPatchFcnHdl)


