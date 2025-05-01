import pandas as pd
import os as os
from datetime import datetime
import src.utils.dataStructures as dataStruct


def runExport(modelsBus, fswBus, simParam):
    # Export output data
    dictDataExport = {}
         
    # Time
    dictDataExport.update({"time": simParam.timeVec})

    # Add signals to export
    print("   Exporting saved data :")
    for path in simParam.runOptions.signalPathsToExport:
        sep = "/"
        pathSplit = path.split(sep)
        if (pathSplit[0] == fswBus.name):
            signalObj = dataStruct.getSignalObjFromPath(fswBus, path)
            dictDataExport.update(signalObj.timeseries.toDic())
        elif (pathSplit[0] == modelsBus.name):
            signalObj = dataStruct.getSignalObjFromPath(modelsBus, path)
            dictDataExport.update(signalObj.timeseries.toDic())
        print(f"      {signalObj.name}")

    # Build dataframe
    df = pd.DataFrame(dictDataExport)
         
    # Setup destination folder
    exportDataFolderPath = os.path.join("bin", simParam.caseName)
    try:
        os.mkdir(exportDataFolderPath)
        print(f"   Directory '{exportDataFolderPath}' created successfully.")
    except FileExistsError:
        print(f"   Directory '{exportDataFolderPath}' already exists.")
    except PermissionError:
        print(f"   Permission denied: Unable to create '{exportDataFolderPath}'.")
    except Exception as e:
        print(f"   An error occurred: {e}")

    # Setup filename
    nowDt = datetime.now()
    timeStamp = (nowDt.strftime("%Y_%m_%d_%H_%M_%S"))
    exportDataFileName = ("outputData_" + timeStamp + ".csv")

    # Save dataframe   
    exportDataPath = os.path.join(exportDataFolderPath, exportDataFileName)
    print(f"   Output data saved in : {exportDataPath}")
    df.to_csv(exportDataPath)