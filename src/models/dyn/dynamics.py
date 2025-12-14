
import numpy as np

import src.models.dyn.attitudeDynamics as attDyn
import src.models.dyn.orbitDynamics as orbDyn

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# DICTIONNARIES
# --------------------------------------------------
# ...


# --------------------------------------------------
# DATA CLASSES
# --------------------------------------------------
# ...


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# Dynamics model
class Dynamics:
    def __init__(self):
        self.attitude = attDyn.Attitude()
        self.orbit = orbDyn.Orbit()
        # Interface
        self.interface = {}
        self.interface.update({"torqueTot_B": np.array([0, 0, 0])})
        self.interface.update({"orbit": self.orbit.interface})
        self.interface.update({"attitude": self.attitude.interface})

    def run(self, simParam, spacecraft, actIf, envIf ):
        # Compute total external torque
        torqueTot_B = self.computeTotTorque(actIf, envIf)
        # Run attitude dynamics model
        self.attitude.run(simParam, spacecraft, self.interface)
        # Run orbit dynamics model
        self.orbit.run(simParam)

    def computeTotTorque(self, actIf, envIf):
        # Fetch actuators and environment interfaces
        torqueAct_B = actIf["torqueAct_B"]
        torqueEnv_B = envIf["torqueEnv_B"]
        # Process
        torqueTot_B = torqueAct_B + torqueEnv_B
        # Update interface
        self.interface["torqueTot_B"] = torqueTot_B
