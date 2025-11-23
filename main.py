import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from src.data.scenarios.scenarioDefinitions import AVAILABLE_SCENARIOS
from src.utils.runSimu import runSimu
import src.utils.nonRegression as nonRegression 

import src.models.act.thrModels as thrModels

# thrModel = thrModels.Thr()
# thrModel.addThrSet("branchA", 10)
# thrModel.addThrSet("branchB", 10)
# thrModel.addThrSet("branchC", 10)

# keys = thrModel.sets["branchA"].units.keys()
# for key in thrModel.sets["branchA"].units.keys():
# 	thrUnit = thrModel.sets["branchA"].units[key]
# 	print((thrUnit.name, thrUnit.id))

# Run the simulation
scenarioPatchFcnHdl = AVAILABLE_SCENARIOS["testDevelopment"]
out = runSimu(scenarioPatchFcnHdl)

# Run non regression output
# nonRegressionOutput = nonRegression.run(scenarioPatchFcnHdl)
# fswBus/modeMgt/aocsMode
# print(out["fwsBus"].subBuses["modeMgt"].signals["aocsMode"].timeseries.dataVec)
print(out["modelsBus"].subBuses["actuators"].signals["torqueAct_B"].timeseries.dataVec)
