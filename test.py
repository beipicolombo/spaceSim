# -*- coding: utf-8 -*-
"""
Created on Tue Dec 31 14:03:34 2024

@author: Xela
"""

import numpy as np
import src.utils.constants as const
import src.models.scModel as scModel
import src.fsw.fswModel as fswModel
# import src.fsw.fswModeMgt as fswModeMgt
# import src.fsw.fswGuidance as fswGuidance 

# Retrieve useful constants
pi  = np.pi
deg2rad = const.deg2rad


# [Guidance] Simulation-specific parameter
guidParamToPatch = {}
guidParamToPatch["GUIDMODE_ATT_INERT_eulerAngGuid_RI"] = np.array([25, 10, 0]) * deg2rad 
guidParamToPatch["orbitRate"] = 0.8


# Initialize default parameters
scParam = scModel.Spacecraft()
fswParam = fswModel.Fsw(scParam)

# [Guidance] Patch simulation-specific parameters
fswParam.guidParam.GUIDMODE_ATT_INERT_eulerAngGuid_RI = np.array([0, 10, 0]) * deg2rad * 0
   
    
    
def patchParameters(obj, dicToPatch):
    objNew = obj
    for key, value in dicToPatch.items():
        # print(key, value)
        setattr(obj, key, value)
    return objNew

guidParamPatched = patchParameters(fswParam.guidParam, guidParamToPatch)
print(guidParamPatched.orbitRate)
print(guidParamPatched.GUIDMODE_ATT_INERT_eulerAngGuid_RI)



    