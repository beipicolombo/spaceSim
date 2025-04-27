

import pandas as pd
import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from runSimu import *

scenarioPatchFcnHdl = scenarioDefinitions.scenarioDefinition_testDevelopment
(modelsBus, fswBus, simBus) = runSimu(scenarioPatchFcnHdl)

 
     
# list of name, degree, score
time 		 = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.timeVec
angRate_BI_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.dataVec[:,1]

# dictionary of lists
dict = {"time": time,"angRate_BI_B": angRate_BI_B}
     
df = pd.DataFrame(dict)
     
# saving the dataframe
df.to_csv('test.csv')
