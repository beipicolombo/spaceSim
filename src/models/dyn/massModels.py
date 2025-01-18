import numpy as np

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

def getUniformCylinderInertia(mass, rad, length):
	Ixx = 1/2 * mass * rad^2
	Iyy = 1/4 * mass * rad^2 + 1/12 * mass * length^2
	Izz = 1/4 * mass * rad^2 + 1/12 * mass * length^2

	return inertia


def getUniformRectanglePrismInertia(mass, lx, ly, lz):
	Ixx = 1/12 * mass * (lz**2 + ly**2)
	Iyy = 1/12 * mass * (lx**2 + lz**2)
	Izz = 1/12 * mass * (lx**2 + ly**2)
	inertia = np.diag([Ixx, Iyy, Izz])
	
	return inertia
	

