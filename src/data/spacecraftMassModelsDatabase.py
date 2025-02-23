import numpy as np
import src.models.dyn.massModels as massModels


def getSpaceceraftMassModel(spacecraftName = ""):
	# Initialize output
	massParam = massModels.MassParam()

	# Switch depending on the specified spacecraft name
	if (spacecraftName == "spacecraft1"):
		# Mass
		massSc = 1000 # [kg]
		# Construct inertia
		radius = 2 # [m]
		length = 4 # [m]
		inertialSc_G = massModels.getUniformCylinderInertia(massSc, radius, length)
		# Position of CoM
		posComSc_G = np.zeros(3)

	elif (spacecraftName == "spacecraft2"):
		# Mass
		massSc = 1000 # [kg]
		# Construct inertia
		lx = 2 # [m]
		ly = 2 # [m]
		lz = 4 # [m]
		inertialSc_G = massModels.getUniformRectanglePrismInertia(massSc, lx, ly, lz)
		# Position of CoM
		posComSc_G = np.zeros(3)

	else:
		print("		Specified spacecraft name not recognized, default mass parameters are used")

	# TBW: add computation for center of mass
	inertiaSc_B = inertialSc_G

	massParam.inertialSc_G = inertialSc_G
	massParam.inertiaSc_B = inertiaSc_B
	massParam.inertiaScInv_B = np.linalg.inv(inertiaSc_B)
	massParam.massSc = massSc
	massParam.massCB = massSc

	return massParam