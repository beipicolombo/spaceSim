
import numpy as np

import src.models.scModel as scModel
import src.models.spaceEnv.envModel as envModel
import src.models.dyn.attitudeDynamics as attDyn
import src.models.dyn.orbitDynamics as orbDyn


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# MODELS
# --------------------------------------------------
class Models():
	def __init__(self, massModelName, allParam):
		self.spacecraft = scModel.Spacecraft(massModelName, allParam)
		self.environment = envModel.Environment()
		self.attitude = attDyn.Attitude()
		self.orbit = orbDyn.Orbit()

	def run(self, simParam, simBus, fswBus, modelsBus):
		# Initialize output bus
		modelsBusOut = modelsBus
		# Run environment model
		modelsBusOut.subBuses["environment"] = self.environment.run(simParam, self.spacecraft.massModel, simBus, modelsBus)
		# Run spacecraft model
		modelsBusOut.subBuses["actuators"] = self.spacecraft.run(simParam, fswBus, modelsBus)
		# Compute total external torque
		modelsBusOut = self.computeTotTorque(modelsBusOut)
		# Run dynamics model
		# ...
		# Run sensors model
		# ...

		return modelsBusOut


    # Total total torque
	def computeTotTorque(self, modelsBus):
        # Retrieve useful inputs
		torqueExt_B = modelsBus.subBuses["environment"].signals["torqueExt_B"].value
		torqueAct_B = modelsBus.subBuses["actuators"].signals["torqueAct_B"].value
		# Initialize output bus
		modelsBusOut = modelsBus
		# Compute total external torque
		torqueTot_B = torqueExt_B + torqueAct_B
		# Update output bus signals
		modelsBusOut.subBuses["dynamics"].subBuses["attitude"].signals["torqueTot_B"].update(torqueTot_B)
		return modelsBusOut