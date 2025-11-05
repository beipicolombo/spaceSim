
import os as os
import numpy as np
import pandas as pd

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from src.utils.runSimu import runSimu


# Function to load the reference data
def loadSavedData(caseName, timeStamp = ""):
    print("   Loading saved data :")
	# Set the source path
    sourceDataFolderName = os.path.join("bin", caseName)
    if (len(timeStamp) == 0):
        sourceDataFileName = ("referenceData.csv")
    else:
        sourceDataFileName = ("outputData_" + timeStamp + ".csv")
    sourceDataPth = os.path.join(sourceDataFolderName, sourceDataFileName)
    print(f"   Reference loaded from : {sourceDataPth}")
    df = pd.read_csv(sourceDataPth)

    return df


# Function to compare the two dictionnaries
def checkResults(dictDataRef, dictDataExport):
    isOkAll = True
    regResults = {}
    for key in dictDataRef:
        # TBW: check time
        # TBW: check artifact from dataframe / dictionnary conversion
        # TBW: check parameters
        
        eps = 1e-9
        
        if (key != "Unnamed: 0") and (key != "time"):
            isOk = (np.max(abs(np.array(dictDataRef[key])-np.array(dictDataExport[key]))) < eps)
            isOkAll = isOk and isOkAll
            regResults.update({key: isOk})
            
            print(("   " + key + " : " + str(isOk)))
    if isOkAll:
        print("   Non regression test was successful")
    else:
        print("   Non regression test failed")
        
    return (regResults, isOkAll)



# Function to run the non-regression test
def run(scenarioPatchFcnHdl):
    # 1. Run the simulation
    simOutputs = runSimu(scenarioPatchFcnHdl) 
    dicData = simOutputs["dictDataExport"]
    simParam = simOutputs["simParam"]
    
    # 2. Retrieve the reference data
    print("==================================")
    print("Retrieve reference data")
    refCaseName = simParam.caseName
    dictDataRef = loadSavedData(refCaseName).to_dict("list")
    
	# 3. Compare results
    print("==================================")
    print("Checking simulations results with reference")
    (regResults, isAllOk) = checkResults(dictDataRef, dictData)

    # 4. Set outputs
    outputDict = {"regResults": regResults, "isAllOk": isAllOk}
    return outputDict