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
    
    def normalize(self):
        quatNormalized = Quaternion()
        quatNorm = self.norm()
        quatNormalized.sca = self.sca / quatNorm
        quatNormalized.vec = self.vec / quatNorm
        return quatNormalized

    def conjugate(self):
        conjQuat = Quaternion()
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
        sca = self.sca
        vec = self.vec        
        phi   = m.atan2(2 * (qw*qx + qy*qz), (1 - 2*(qx**2+qy**2)))
        theta = m.asin(2 * (qw*qy - qz*qx)) 
        psi   = m.atan2(2 * (qw*qz + qx*qy), (1 - 2*(qy**2+qz**2)))
        eulerAngles = np.array([phi, theta, psi])
        return eulerAngles

    def toDcm(self):
        # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        qw = self.sca
        qx = self.vec[0]
        qy = self.vec[1]
        qz = self.vec[2]
        M11 = qw**2 + qx**2 - qy**2 - qz**2
        M12 = 2*(qx*qy - qw*qz)
        M13 = 2*(qw*qy + qw*qz)
        M21 = 2*(qx*qy + qw*qz)
        M22 = qw**2 - qx**2 + qy**2 - qz**2
        M23 = 2*(qy*qz - qw*qx)
        M31 = 2*(qx*qz - qw*qy)
        M32 = 2*(qw*qx + qy*qz)
        M33 = qw**2 - qx**2 - qy**2 + qz**2
        dcm = np.array([[M11, M12, M13], [M21, M22, M23], [M31, M32, M33]])
        return dcm

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
        
        quatNext = trans_VecToQuat(xNext, True)
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
def applyRotation(qAB, v_B):
    quat_v_B = Quaternion(0, v_B)
    quat_v_A = multiplyQuat(multiplyQuat(qAB, quat_v_B), qAB.conjugate())
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
    crossMat[0,1] = vec[2]
    crossMat[0,2] = -vec[1]
    crossMat[1,2] = vec[0]
    crossMat[1,0] = -vec[2]
    crossMat[2,0] = vec[1]
    crossMat[2,1] = -vec[0]
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