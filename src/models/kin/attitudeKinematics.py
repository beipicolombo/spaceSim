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

    def random():
        quat = Quaternion(np.random.rand(), np.random.rand(3))
        return quat
    
    def norm(self):
        return np.linalg.norm(self.toVec())

    def validate_norm(self):
        quat = Quaternion()
        vector = np.append([quat.sca], quat.vec)
        vectorNorm = np.linalg.norm(vector)
        isOk = ((quat.norm()-vectorNorm) < 1e-10)
        return {"isOk": True}

    def isIdentity(self):
        vecnorm = np.linalg.norm(self.vec)
        eps = 1e-10
        isIdentity = ((vecnorm < eps) and ((abs(self.sca)-1)<eps))
        return isIdentity

    def validate_isIdentity(self):
        q1 = Quaternion()
        q1.sca = 1
        q1.vec = np.array([0, 0, 0])
        q2 = Quaternion()
        q2.sca = 2.0
        q2.vec = np.array([0, 0, 0])
        q3 = Quaternion()
        q3.sca = 1.0
        q3.vec = np.array([0, 0.1, 0])
        q4 = Quaternion.random()
        isOk = q1.isIdentity() and not(q2.isIdentity()) and not(q3.isIdentity()) and not(q4.isIdentity())
        return {"isOk": True}

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

    def validate_normalize(self):
        q1 = Quaternion.random()
        q1 = q1.normalize()   
        vector = np.append([q1.sca], q1.vec)
        vectorNorm = np.linalg.norm(vector)
        isOk = abs(vectorNorm - 1) < 1e-10
        return {"isOk": True}

    def conjugate(self):
        conjQuat = Quaternion()
        conjQuat.sca = self.sca
        conjQuat.vec = -self.vec
        return conjQuat

    def validate_conjugate(self):
        q1 = Quaternion.random()
        q2 = q1.conjugate()
        isOk = (q2.sca == q1.sca) and all(q1.vec == -q2.vec)
        return {"isOk": True}

    def toVec(self):
        return np.append([self.sca], self.vec)

    def validate_toVec(self):
        testVec = np.random.rand(4)
        q1 = Quaternion(testVec[0], testVec[1:])
        isOk = (max(abs(q1.toVec() - testVec)) < 1e-10)
        return {"isOk": True}
    
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

    def validate_toEuler(self):
        # To be implemented
        isOk = True
        return {"isOk": isOk}

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

    def validate_toDcm(self):
        eulerAngRef = np.array([-10, -90, 28]) * deg2rad # 321 Euler angle sequence
        eulerAngRef = (np.random.rand(3)*2 - np.ones(3)) *90 *  deg2rad
        Mx = Ry(eulerAngRef[0])
        My = Ry(eulerAngRef[1])
        Mz = Ry(eulerAngRef[2])
        M = np.matmul(Mx, np.matmul(My, Mz)) 
        eps = 1e-10
        isOk1 = (np.max(abs(Mx - trans_DcmToQuat(Mx).toDcm())) < eps)
        isOk2 = (np.max(abs(My - trans_DcmToQuat(My).toDcm())) < eps)
        isOk3 = (np.max(abs(Mz - trans_DcmToQuat(Mz).toDcm())) < eps)
        isOk4 = (np.max(abs(M - trans_DcmToQuat(M).toDcm())) < eps)
        isOk = (isOk1 and isOk2 and isOk3 and isOk4)
        return {"isOk": isOk}

    def toVecRot(self):
        eps = 1e-10
        if not self.isIdentity():
            ang0 = 2*np.arccos(self.sca)
            if (abs(ang0) < eps) or (abs(ang0-2*np.pi) < eps):
                ang = 0
                vec = np.array([1, 0, 0]) 
            elif (ang0 < np.pi):
                ang = ang0
                vec = self.vec/np.sin(ang/2)
            else:
                ang = ang0 - 2*np.pi # take a negative angle
                vec = self.vec/abs(np.sin(ang/2)) # keep the same direction
        else:
            vec = np.array([1, 0, 0])
            ang = 0
        return (vec, ang)

    def validate_toVecRot(self):
        # To be implemented
        isOk = True
        return {"isOk": isOk}

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

    def validate_propagateState(self):
        # To be implemented
        isOk = True
        return {"isOk": isOk}
     
    
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


def validate_multiplyQuat():
    # To be implemented
    isOk = True
    return {"isOk": isOk}  


# Quaternion rotation
def applyRotation(qBA, v_B):
    quat_v_B = Quaternion(0, v_B)
    quat_v_A = multiplyQuat(multiplyQuat(qBA, quat_v_B), qBA.conjugate())
    return quat_v_A.vec


def validate_applyRotation():
    # To be implemented
    isOk = True
    return {"isOk": isOk}  


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


def validate_trans_VecRotToQuat():
    # To be implemented
    isOk = True
    return {"isOk": isOk}    


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


def validate_trans_VecToQuat():
    vec = np.random.rand(4)
    quat = trans_VecToQuat(vec, isScalarFirst = True)
    eps = 1e-10
    isQuatScaOk = (abs(vec[0] - quat.sca) < eps)
    isQuatVecOk = np.max(abs(vec[1:] - quat.vec) < eps)
    isOk = isQuatScaOk and isQuatVecOk
    return {"isOk": isOk}     


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


def validate_trans_VecToCrossMat():
    vec1 = np.random.rand(3)
    vec2 = np.random.rand(3)
    Mcross = trans_VecToCrossMat(vec1)
    eps = 1e-10
    isOk = np.max(abs(np.matmul(Mcross, vec2) - np.cross(vec1, vec2)) < eps)
    return {"isOk": isOk}   


# Rotation (vector + angle) to quaternion
def trans_AngVecToQuat(angle, vec):
    quatSca = m.cos(angle/2)
    quatVec = m.sin(angle/2)*np.array(vec)
    quatOut = Quaternion(quatSca,quatVec)
    return quatOut


def validate_trans_AngVecToQuat():
    ang = (2*np.random.rand()-1) * 180 * deg2rad
    vec = np.random.rand()
    vec = vec / np.linalg.norm(vec) # normalize
    quat = trans_AngVecToQuat(ang, vec)
    eps = 1e-10
    isQuatScaOk = (abs(np.cos(ang/2) - quat.sca) < eps)
    isQuatVecOk = (np.max(abs(np.sin(ang/2)*vec - quat.vec)) < eps)
    isOk = isQuatScaOk and isQuatVecOk
    return {"isOk": isOk}


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


def validate_trans_EulerAngToQuat():
    # To be implemented
    isOk = True
    return {"isOk": isOk}


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


def validate_trans_DcmToEulerAng():
    # To be implemented
    isOk = True
    return {"isOk": isOk}


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


def validate_trans_DcmToQuat():
    # To be implemented
    isOk = True
    return {"isOk": isOk}


# 321 euler angles to DCM
def trans_EulerAngToDcm(eulerAngles):
    dcm = Rx(eulerAngles[0]) * Ry(eulerAngles[1]) * Rz(eulerAngles[2])
    return dcm


def validate_trans_EulerAngToDcm():
    # To be implemented
    isOk = True
    return {"isOk": isOk}    


# X axis rotation DCM
def Rx(ang):
    return np.array([[1, 0, 0], [0, np.cos(ang), np.sin(ang)], [0, -np.sin(ang), np.cos(ang)]])


def validate_Rx():
    ang = (2*np.random.rand()-1) * deg2rad
    R = Rx(ang)
    Rprod1 = np.matmul(R, np.transpose(R))
    Rprod2 = np.matmul(np.transpose(R), R)
    eps = 1e-10
    isInvOk = (np.max(abs(Rprod1 - np.eye(3))) < eps) and (np.max(abs(Rprod2 - np.eye(3))) < eps)
    isAxOk = (R[0,0]==1 and R[1,1] == np.cos(ang) and R[1,2] == np.sin(ang))
    isOk = isInvOk and isAxOk
    return {"isOk": isOk}  


# Y axis rotation DCM
def Ry(ang):
    return np.array([[np.cos(ang), 0, -np.sin(ang)], [0, 1, 0], [np.sin(ang), 0, np.cos(ang)]])


def validate_Ry():
    ang = (2*np.random.rand()-1) * deg2rad
    R = Ry(ang)
    Rprod1 = np.matmul(R, np.transpose(R))
    Rprod2 = np.matmul(np.transpose(R), R)
    eps = 1e-10
    isInvOk = (np.max(abs(Rprod1 - np.eye(3))) < eps) and (np.max(abs(Rprod2 - np.eye(3))) < eps)
    isAxOk = (R[1,1]==1 and R[0,0] == np.cos(ang) and R[0,2] == -np.sin(ang))
    isOk = isInvOk and isAxOk
    return {"isOk": isOk}  


# Z axis rotation DCM
def Rz(ang):
    return np.array([[np.cos(ang), np.sin(ang), 0], [-np.sin(ang), np.cos(ang), 0], [0, 0, 1]])


def validate_Rz():
    ang = (2*np.random.rand()-1) * deg2rad
    R = Rz(ang)
    Rprod1 = np.matmul(R, np.transpose(R))
    Rprod2 = np.matmul(np.transpose(R), R)
    eps = 1e-10
    isInvOk = (np.max(abs(Rprod1 - np.eye(3))) < eps) and (np.max(abs(Rprod2 - np.eye(3))) < eps)
    isAxOk = (R[2,2]==1 and R[0,0] == np.cos(ang) and R[0,1] == np.sin(ang))
    isOk = isInvOk and isAxOk
    return {"isOk": isOk}  


# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------