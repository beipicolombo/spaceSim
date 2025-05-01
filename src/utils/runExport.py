import pandas as pd
import src.utils.dataStructures as dataStruct


def runExport(modelsBus, fswBus, simParam):
    # Export output data
    dictDataExport = {}
         
    # Time
    dictDataExport.update({"time": simParam.timeVec})

    # Add signals to export
    for path in simParam.runOptions.signalPathsToExport:
    	sep = "/"
    	pathSplit = path.split(sep)
    	if (pathSplit[0] == fswBus.name):
    		dictDataExport.update(dataStruct.getSignalObjFromPath(fswBus, path).timeseries.toDic())
    	elif (pathSplit[0] == modelsBus.name):
    		dictDataExport.update(dataStruct.getSignalObjFromPath(modelsBus, path).timeseries.toDic())
         
    df = pd.DataFrame(dictDataExport)
         
    # saving the dataframe
    df.to_csv('test.csv')