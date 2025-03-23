import numpy as np
import src.models.dyn.massModels as massModels


def getSpaceceraftMassModel(massModelName = "Default"):
	# Initialize output
	massParam = massModels.MassParam()

	# Switch depending on the specified spacecraft name
	if (massModelName == "SC_1000kg_cyl"):
		# Mass
		massSc = 1000 # [kg]
		# Construct inertia
		radius = 2 # [m]
		length = 4 # [m]
		inertialSc_G = massModels.getUniformCylinderInertia(massSc, radius, length)
		# Position of CoM
		posComSc_G = np.zeros(3)

	elif (massModelName == "SC_1000kg_rect"):
		# Mass
		massSc = 1000 # [kg]
		# Construct inertia
		lx = 4 # [m]
		ly = 2 # [m]
		lz = 2 # [m]
		inertialSc_G = massModels.getUniformRectanglePrismInertia(massSc, lx, ly, lz)
		# Position of CoM
		posComSc_G = np.zeros(3)

	elif (massModelName == "SC_100kg_rect"):
		# Mass
		massSc = 100 # [kg]
		# Construct inertia
		lx = 1 # [m]
		ly = 0.5 # [m]
		lz = 0.5 # [m]
		inertialSc_G = massModels.getUniformRectanglePrismInertia(massSc, lx, ly, lz)
		# Position of CoM
		posComSc_G = np.zeros(3)
		
	elif (massModelName == "Default"):
		# Mass
		massSc = 100 # [kg]
		# Construct inertia
		lx = 1 # [m]
		ly = 0.5 # [m]
		lz = 0.5 # [m]
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