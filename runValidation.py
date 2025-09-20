

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from runSimu import *
import sys as sys


def main():
	try:
		# Run the simulation
		scenarioPatchFcnHdl = scenarioDefinitions.scenarioDefinition_manualModeDev
		# scenarioPatchFcnHdl = scenarioDefinitions.scenarioDefinition_nominalScenario
		(dictDataExport, simParam) = runSimu(scenarioPatchFcnHdl)

		# Check for termination condition
		user_input = input("Exit program? (y/n): ")
		if user_input.lower() == "y":
			sys.exit(0)

	except Exception as e:
	    print(f"An error occurred: {e}")
	    sys.exit(0)


if __name__ == "__main__":
	main()
