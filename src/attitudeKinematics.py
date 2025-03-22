# -*- coding: utf-8 -*-
"""
Created on Sun Mar 27 13:15:55 2022

@author: Xela
"""

import numpy as np
import math as m

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------

# Quaternion
class Quaternion:
    def __init__(self, quatSca=1, quatVec=np.zeros(3)):    
        self.sca = quatSca
        self.vec = quatVec
    
    def norm(self):
        return np.linalg.norm(self.toVec())

    def isIdentity(self):
        vecnorm = np.linalg.norm(self.vec)
        eps = 1e-10
        isIdentity = ((vecnorm < eps) and ((self.sca-1)<eps))
        return isIdentity
    
    def normalize(self):
        quatNormalized = Quaternion()
        if not self.isIdentity():
            quatNorm = self.norm()
            quatNormalized.sca = self.sca / quatNorm
            quatNormalized.vec = self.vec / quatNorm
        else:
            # Identity quaternion
            quatNormalized = self
        return quatNormalized

    def conjugate(self):
        conjQuat = Quaternion()
        conjQuat.sca = self.sca
        conjQuat.vec = -self.vec
        return conjQuat
    
    def toVec(self):
        return np.append([self.sca], self.vec)  
    
    def toEuler(self):
        # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        qw = self.sca
        qx = self.vec[0]
        qy = self.vec[1]
        qz = self.vec[2]     
        phi   = m.atan2(2 * (qw*qx + qy*qz), (1 - 2*(qx**2+qy**2)))
        theta = m.asin(2 * (qw*qy - qz*qx)) 
        psi   = m.atan2(2 * (qw*qz + qx*qy), (1 - 2*(qy**2+qz**2)))
        # Handle gimbal lock
        if (abs(theta-pi/2) < 1e-10):
            phi = 0
            psi = -2*m.atan2(qx, qw)
        elif (abs(theta+pi/2) < 1e-10):
            phi = 0
            psi = 2*m.atan2(qx, qw)
        
        eulerAngles = np.array([phi, theta, psi])
        return eulerAngles

    def toDcm(self):
        # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        # Source: https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran
        qw = self.sca
        qx = self.vec[0]
        qy = self.vec[1]
        qz = self.vec[2]
        M11 = qw**2 + qx**2 - qy**2 - qz**2
        M12 = 2*(qx*qy + qz*qw)
        M13 = 2*(qx*qz - qy*qw)
        M21 = 2*(qx*qy - qz*qw)
        M22 = qw**2 - qx**2 + qy**2 - qz**2
        M23 = 2*(qy*qz + qx*qw)
        M31 = 2*(qx*qz + qy*qw)
        M32 = 2*(qy*qz - qx*qw)
        M33 = qw**2 - qx**2 - qy**2 + qz**2
        dcm = np.array([[M11, M12, M13], [M21, M22, M23], [M31, M32, M33]])
        return dcm

    def toVecRot(self):
        if not self.isIdentity():
            vec = self.vec/np.linalg.norm(self.vec)
            ang = 2*np.arccos(self.sca)
        else:
            vec = np.array([1, 0, 0])
            ang = 0
        return (vec, ang)

    def propagateState(self, angleRates_B, simParam):
        xPrev = self.toVec()
        Ts = simParam.Ts
        
        k1 = quatDot(self, angleRates_B).toVec()
        x2 = xPrev + k1/2
        k2 = quatDot(trans_VecToQuat(x2, True), angleRates_B).toVec()
        x3 = xPrev + k2/2
        k3 = quatDot(trans_VecToQuat(x3, True), angleRates_B).toVec()
        x4 = xPrev + k3
        k4 = quatDot(trans_VecToQuat(x4, True), angleRates_B).toVec()
        xNext = xPrev + Ts/6 * (k1 + 2*k2 + 2*k3 + k4)
        
        quatNext = trans_VecToQuat(xNext, True).normalize()
        self.sca = quatNext.sca
        self.vec = quatNext.vec
     
    
# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

# Quaternion multiplication
def multiplyQuat(quat1, quat2):
    quatProd = Quaternion()
    sca1 = quat1.sca
    sca2 = quat2.sca
    vec1 = quat1.vec
    vec2 = quat2.vec
    quatProd.sca = sca1*sca2 - np.dot(vec1, vec2)  
    quatProd.vec = sca1*vec2 + sca2*vec1 + np.cross(vec1, vec2) 
    return quatProd


# Quaternion rotation
def applyRotation(qBA, v_B):
    quat_v_B = Quaternion(0, v_B)
    quat_v_A = multiplyQuat(multiplyQuat(qBA, quat_v_B), qBA.conjugate())
    return quat_v_A.vec


# Quaternion derivative
def quatDot(quat, angleRates):
    quatDot = Quaternion()
    quatSca = quat.sca
    quatVec = quat.vec    
    angleRates = np.array(angleRates)
    quatDot.vec = 1/2 * (quatSca * angleRates + np.cross(quatVec, angleRates))
    quatDot.sca = -1/2 * np.dot(quatVec, angleRates)
    return quatDot
    

# Rotation (vector + angle) to quaternion
def trans_VecRotToQuat(vec, angle):
    sca = np.cos(angle/2)
    vec = np.sin(angle/2)*vec    
    quatOut = Quaternion(sca,vec).normalize()
    return quatOut


# Vector to quaternion
def trans_VecToQuat(vec, isScalarFirst):
    if isScalarFirst:
        quatSca = vec[0]
        quatVec = vec[1:4]
    else:
        quatSca = vec[3]
        quatVec = vec[0:3]    
    quatOut = Quaternion(quatSca,quatVec)
    return quatOut


# Vector to skew matrix
def trans_VecToCrossMat(vec):
    crossMat = np.zeros((3,3))
    crossMat[0,1] = -vec[2]
    crossMat[0,2] = vec[1]
    crossMat[1,2] = -vec[0]
    crossMat[1,0] = vec[2]
    crossMat[2,0] = -vec[1]
    crossMat[2,1] = vec[0]
    return crossMat        


# Rotation (vector + angle) to quaternion
def trans_AngVecToQuat(angle, vec):
    quatSca = m.cos(angle/2)
    quatVec = m.sin(angle/2)*np.array(vec)
    quatOut = Quaternion(quatSca,quatVec)
    return quatOut

# 321 euler angles to quaternion
def trans_EulerAngToQuat(eulerAngles):
    # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # Euler angles vectors: [0] roll , [1] pitch, [2] yaw
    cr = np.cos(eulerAngles[0]/2)
    cp = np.cos(eulerAngles[1]/2)
    cy = np.cos(eulerAngles[2]/2)
    sr = np.sin(eulerAngles[0]/2)
    sp = np.sin(eulerAngles[1]/2)
    sy = np.sin(eulerAngles[2]/2)
    quat = Quaternion()
    quat.sca    = cr*cp*cy + sr*sp*sy
    quat.vec[0] = sr*cp*cy - cr*sp*sy
    quat.vec[1] = cr*sp*cy + sr*cp*sy
    quat.vec[2] = cr*cp*sy - sr*sp*cy

    return quat

# DCM to 321 euler angles
def trans_DcmToEulerAng(dcm):
    M11 = dcm[0, 0]
    M12 = dcm[0, 1]
    M13 = dcm[0, 2]
    M21 = dcm[1, 0]
    M22 = dcm[1, 1]
    M23 = dcm[1, 2]
    M31 = dcm[2, 0]
    M32 = dcm[2, 1]
    M33 = dcm[2, 2]

    phi = m.atan2(M12, M11)
    theta = -m.asin(M13)
    psi = m.atan2(M23, M33)
    eulerAngles = np.array([phi, theta, psi])

    return eulerAngles

# DCM to quaternion
def trans_DcmToQuat(dcm):
    # Source: https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran
    quat = Quaternion()
    M11 = dcm[0, 0]
    M12 = dcm[0, 1]
    M13 = dcm[0, 2]
    M21 = dcm[1, 0]
    M22 = dcm[1, 1]
    M23 = dcm[1, 2]
    M31 = dcm[2, 0]
    M32 = dcm[2, 1]
    M33 = dcm[2, 2]

    qw2 = 1/4 * (1 + M11 + M22 + M33)
    qx2 = 1/4 * (1 + M11 - M22 - M33)
    qy2 = 1/4 * (1 - M11 + M22 - M33)
    qz2 = 1/4 * (1 - M11 - M22 + M33)

    idxMax = np.argmax([qx2, qy2, qz2, qw2])

    if (idxMax == 0):
        quat.vec = 1/(4*np.sqrt(qx2)) * np.array([4*qx2, M12+M21, M31+M13])
        quat.sca = 1/(4*np.sqrt(qx2)) * (M23 - M32)

    elif (idxMax == 1):
        quat.vec = 1/(4*np.sqrt(qy2)) *  np.array([M12+M21, 4*qy2, M23+M32])
        quat.sca = 1/(4*np.sqrt(qy2)) * (M31 - M13)  

    elif (idxMax == 2):
        quat.vec = 1/(4*np.sqrt(qz2)) * np.array([M31+M13, M23+M32, 4*qz2])
        quat.sca = 1/(4*np.sqrt(qz2)) * (M12 - M21)

    else:
        quat.vec = 1/(4*np.sqrt(qw2)) * np.array([M23-M32, M31-M13, M12-M21])
        quat.sca = np.sqrt(qw2)

    
    return quat

# 321 euler angles to DCM
def trans_EulerAngToDcm(eulerAngles):
    dcm = Rx(eulerAngles[0]) * Ry(eulerAngles[1]) * Rz(eulerAngles[2])
    return dcm


# X axis rotation DCM
def Rx(ang):
    return np.array([[1, 0, 0], [0, np.cos(ang), np.sin(ang)], [0, -np.sin(ang), np.cos(ang)]])


# Y axis rotation DCM
def Ry(ang):
    return np.array([[np.cos(ang), 0, -np.sin(ang)], [0, 1, 0], [np.sin(ang), 0, np.cos(ang)]])


# Z axis rotation DCM
def Rz(ang):
    return np.array([[np.cos(ang), np.sin(ang), 0], [-np.sin(ang), np.cos(ang), 0], [0, 0, 1]])

# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------