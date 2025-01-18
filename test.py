# -*- coding: utf-8 -*-
"""
Created on Tue Dec 31 14:03:34 2024

@author: Xela
"""

import src.models.act.thrModels as thrModels
import src.models.act.rwModels as rwModels
import src.attitudeDynamics as attitudeDynamics


# attDynParam = attitudeDynamics.AttDynParam()
# thrModelParam = thrModels.ThrModelParam(attDynParam)
# thrModelParam.updateThrSets(1, attDynParam)
# thrModelParam.thrSets[0].updateThrSet(3, attDynParam)

rwModelParam = rwModels.RwModelParam()
rwModelParam.updateRwSets(1)
rwModelParam.rwSets[0].updateRwSet(3)