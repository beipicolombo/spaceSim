import numpy as np

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class MassParam:
    def __init__(self):
        # Inertial at the geometric frame origin, in geometric frame => TBW
        self.inertialSc_G = np.identity(3)
        # Inertia at the center of mass, in body frame [kg*m^2]
        self.inertiaSc_B  = np.identity(3)
        self.inertiaScInv_B = np.linalg.inv(self.inertiaSc_B)
        # S/C total mass [kg]
        self.massSc  = 1
        # S/C CB mass [kg] (equal to S/C total mass by default) => appendages to be added
        self.massCB = self.massSc
        # S/C center of mass position in geometric frame [m]
        self.posComSc_G = np.zeros(3)


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

def getUniformCylinderInertia(mass, rad, length):
    Ixx = 1/2 * mass * rad^2
    Iyy = 1/4 * mass * rad^2 + 1/12 * mass * length^2
    Izz = 1/4 * mass * rad^2 + 1/12 * mass * length^2
    inertia = np.diag([Ixx, Iyy, Izz])
    
    return inertia


def getUniformRectanglePrismInertia(mass, lx, ly, lz):
	Ixx = 1/12 * mass * (lz**2 + ly**2)
	Iyy = 1/12 * mass * (lx**2 + lz**2)
	Izz = 1/12 * mass * (lx**2 + ly**2)
	inertia = np.diag([Ixx, Iyy, Izz])

	return inertia
	

