
import numpy as np

import src.utils.paramTools as paramTools


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
# Sensor processing function
class SensorProcessing():
    def __init__(self):
        # Parameters
        self.param = Param()
        # State
        self.state = State()
        # Interface
        self.interface = {}

    # Build 
    def build(self, patches):
        # Do nothing for now
        # Patch parameters
        self.param = Param()
        # Set state
        self.state = State()

    # Run
    def run(self, modeMgtIf):
        # Do nothing for now
        dum = 0


# Sensor processing state
class State():
    def __init__(self):
        # [TBW] Dummy state
        self.dummy = 0


# Sensor processing parameters
class Param():
    def __init__(self):
        # [TBW] Dummy parameters
        self.dummy = 0


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------