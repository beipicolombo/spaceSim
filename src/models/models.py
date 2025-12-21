
import numpy as np

import src.utils.paramTools as paramTools

import src.models.scModel as scModel
import src.models.spaceEnv.envModel as envModel
import src.models.dyn.dynamics as dynModel


# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class Models():
    def __init__(self, massModelName, allParam):
        self.spacecraft = scModel.Spacecraft(massModelName, allParam)
        self.environment = envModel.Environment()
        self.dynamics = dynModel.Dynamics()
        # Interface
        self.interface = {}
        self.interface.update({"spacecraft": self.spacecraft.interface})
        self.interface.update({"environment": self.environment.interface})
        self.interface.update({"dynamics": self.dynamics.interface})

    # Build
    def build(self, patches):
        # Build objects
        self.spacecraft.build(patches)
        self.environment.build(patches)
        self.dynamics.build(patches)

    # Run
    def run(self, simulation, fswIf):
        # Fetch input
        simParam = simulation.param
        simIf = simulation.interface

        # Run spacecraft actuators model
        self.spacecraft.act.run(simParam.simOptions, fswIf)

        # Run dynamics model
        actIf = self.spacecraft.act.interface # From current timestep
        envIfPre = self.environment.interface # From previous timestep
        self.dynamics.run(simParam, self.spacecraft, actIf, envIfPre)

        # Run environment model
        dynIf = self.dynamics.interface # From current timestep
        self.environment.run(simParam, self.spacecraft, simIf, dynIf)

        # Run spacecraft sensors model
        envIf = self.environment.interface # From current timestep
        self.spacecraft.sen.run(fswIf, dynIf, envIf)