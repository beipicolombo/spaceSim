

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from runSimu import *

scenarioPatchFcnHdl = scenarioDefinitions.scenarioDefinition_testDevelopment
(modelsBusm fswBus, simBus) = runSimu(scenarioPatchFcnHdl)
