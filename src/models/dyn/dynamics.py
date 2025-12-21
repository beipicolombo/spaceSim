
import numpy as np

import src.utils.paramTools as paramTools

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
        # Common parameters
        self.param = Param()
        # Common state
        self.state = State()
        # Attitude model
        self.attitude = attDyn.Attitude()
        # Orbit model
        self.orbit = orbDyn.Orbit()
        # Interface
        self.interface = {}
        self.interface.update({"torqueTot_B": np.array([0, 0, 0])})
        self.interface.update({"orbit": self.orbit.interface})
        self.interface.update({"attitude": self.attitude.interface})

    # Build
    def build(self, patches):
        # Patch parameters
        self.param = Param()
        self.orbit.config = paramTools.patchAttributes(self.orbit.config, patches["orbitInitParamPatch"])
        # Set state
        self.state = State()
        self.orbit.set()

    # Run
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


# Parameters
class Param():
    def __init__(self):
        # Dummy parameter
        self.dummy = 0

# State
class State():
    def __init__(self):
        # Dummy state
        self.dummy = 0