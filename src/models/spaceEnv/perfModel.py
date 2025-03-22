# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np
import src.attitudeKinematics as attKin

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def computeBodyNadirAttitude(modelsBus):
	# Initialize output
	modelsBusOut = modelsBus
	
	# Exctract parameters / signals of interest
	qBI_sca = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].value
	qBI_vec = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].value
	angRate_BI_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].value
	qLI_sca = modelsBus.subBuses["environment"].signals["qLI_sca"].value
	qLI_vec = modelsBus.subBuses["environment"].signals["qLI_vec"].value
	angRate_LI_L = modelsBus.subBuses["environment"].signals["angRate_LI_L"].value

	# Process
	qBI = attKin.Quaternion(qBI_sca, qBI_vec)
	qLI = attKin.Quaternion(qLI_sca, qLI_vec)
	qBL = attKin.multiplyQuat(qBI, qLI.conjugate())
	eulerAng_BL = qBL.toEuler()
	angRate_BL_B = angRate_BI_B - np.matmul(qBL.toDcm(), angRate_LI_L)

	# Update output bus signals
	modelsBusOut.subBuses["performance"].signals["qBL_sca"].update(qBL.sca)
	modelsBusOut.subBuses["performance"].signals["qBL_vec"].update(qBL.vec)
	modelsBusOut.subBuses["performance"].signals["eulerAng_BL"].update(eulerAng_BL)
	modelsBusOut.subBuses["performance"].signals["angRate_BL_B"].update(angRate_BL_B)

	return modelsBusOut


# --------------------------------------------------
# SIMULATION / TEST
# -------------------------------------------------- 