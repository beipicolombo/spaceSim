# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 22:06:57 2024

@author: Xela
"""

import numpy as np

deg2rad = np.pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------

class FswEstimationParam:
    # To be moved as higher level class
    def __init__(self):
        self.Dummy = 0


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def attitudeEstimation(fswEstimationParam, fswBus, modelsBus):
    # Retrieve useful inputs
    angRateMeas_BI_B = modelsBus.subBuses["sensors"].signals["angRateMeas_BI_B"].value
    eulerAngMeas_BI = modelsBus.subBuses["sensors"].signals["eulerAngMeas_BI"].value

    # Initialie output bus
    fswEstBusOut = fswBus.subBuses["estimation"]

    # Angular rates estimation
    angRateEst_BI_B = angRateMeas_BI_B
    # Attitude estimation
    eulerAngEst_BI = eulerAngMeas_BI

    # Update output bus signals
    fswEstBusOut.signals["angRateEst_BI_B"].update(angRateEst_BI_B)
    fswEstBusOut.signals["eulerAngEst_BI"].update(eulerAngEst_BI)
    
    return fswEstBusOut


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------

