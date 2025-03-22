# -*- coding: utf-8 -*-
"""
Created on Sun Mar 16 10:47:22 2025

@author: Xela
"""

import numpy as np
import src.attitudeKinematics as attKin

deg2rad = np.pi/180


# --------------------------------------------------
# CLASSES
# --------------------------------------------------

def generateRandomQuaternion():
    quat = attKin.Quaternion(np.random.rand(), np.random.rand(3))
    return quat

# Quaternion
def validate_Quaternion_norm():
    q1 = generateRandomQuaternion()
    vector = np.append([q1.sca], q1.vec)
    vectorNorm = np.linalg.norm(vector)
    isOk = ((q1.norm()-vectorNorm) < 1e-10)
    return isOk

def validate_Quaternion_isIdentity():
    q1 = attKin.Quaternion()
    q1.sca = 1
    q1.vec = np.array([0, 0, 0])
    q2 = attKin.Quaternion()
    q2.sca = 2.0
    q2.vec = np.array([0, 0, 0])
    q3 = attKin.Quaternion()
    q3.sca = 1.0
    q3.vec = np.array([0, 0.1, 0])
    q4 = generateRandomQuaternion()
    isOk = q1.isIdentity() and not(q2.isIdentity()) and not(q3.isIdentity()) and not(q4.isIdentity())
    return isOk

def validate_Quaternion_normalize():
    q1 = generateRandomQuaternion()
    q1 = q1.normalize()   
    vector = np.append([q1.sca], q1.vec)
    vectorNorm = np.linalg.norm(vector)
    isOk = abs(vectorNorm - 1) < 1e-10
    return isOk

def validate_Quaternion_conjugate():
    q1 = generateRandomQuaternion()
    q2 = q1.conjugate()
    isOk = (q2.sca == q1.sca) and all(q1.vec == -q2.vec)
    return isOk

def validate_Quaternion_toVec():
    testVec = np.random.rand(4)
    q1 = attKin.Quaternion(testVec[0], testVec[1:])
    isOk = (max(abs(q1.toVec() - testVec)) < 1e-10)
    return isOk

def validate_Quaternion_toEuler():
    return "TBW"

def validate_Quaternion_toDcm():
    eulerAngRef = np.array([-10, -90, 28]) * deg2rad # 321 Euler angle sequence
    eulerAngRef = (np.random.rand(3)*2 - np.ones(3)) *90 *  deg2rad
    Mx = attKin.Ry(eulerAngRef[0])
    My = attKin.Ry(eulerAngRef[1])
    Mz = attKin.Ry(eulerAngRef[2])
    M = np.matmul(Mx, np.matmul(My, Mz)) 
    eps = 1e-10
    isOk1 = (np.max(abs(Mx - attKin.trans_DcmToQuat(Mx).toDcm())) < eps)
    isOk2 = (np.max(abs(My - attKin.trans_DcmToQuat(My).toDcm())) < eps)
    isOk3 = (np.max(abs(Mz - attKin.trans_DcmToQuat(Mz).toDcm())) < eps)
    isOk4 = (np.max(abs(M - attKin.trans_DcmToQuat(M).toDcm())) < eps)
    isOk = (isOk1 and isOk2 and isOk3 and isOk4)
    return isOk

def validate_Quaternion_toVecRot():
    return "TBW"

def validate_attitudeKinematics_classes():
    print("Validate attitudeKinematics classes...")
    print("   validate_Quaternion_norm:       " + str(validate_Quaternion_norm()))
    print("   validate_Quaternion_isIdentity: " + str(validate_Quaternion_isIdentity()))
    print("   validate_Quaternion_normalize:  " + str(validate_Quaternion_normalize()))
    print("   validate_Quaternion_conjugate:  " + str(validate_Quaternion_conjugate()))
    print("   validate_Quaternion_toVec:      " + str(validate_Quaternion_toVec()))
    print("   validate_Quaternion_toEuler:    " + str(validate_Quaternion_toEuler()))
    print("   validate_Quaternion_toDcm:      " + str(validate_Quaternion_toDcm()))
    print("   validate_Quaternion_toVecRot:   " + str(validate_Quaternion_toVecRot()))
    print("Done")


# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------

def validate_multiplyQuat():
    return "TBW"

def validate_applyRotation():
    return "TBW"

def validate_trans_VecRotToQuat():
    return "TBW"

def validate_trans_VecToQuat():
    vec = np.random.rand(4)
    quat = attKin.trans_VecToQuat(vec, isScalarFirst = True)
    eps = 1e-10
    isQuatScaOk = (abs(vec[0] - quat.sca) < eps)
    isQuatVecOk = np.max(abs(vec[1:] - quat.vec) < eps)
    isOk = isQuatScaOk and isQuatVecOk
    return isOk

def validate_trans_VecToCrossMat():
    vec1 = np.random.rand(3)
    vec2 = np.random.rand(3)
    Mcross = attKin.trans_VecToCrossMat(vec1)
    eps = 1e-10
    isOk = np.max(abs(np.matmul(Mcross, vec2) - np.cross(vec1, vec2)) < eps)
    return isOk

def validate_trans_AngVecToQuat():
    ang = (2*np.random.rand()-1) * 180 * deg2rad
    vec = np.random.rand()
    vec = vec / np.linalg.norm(vec) # normalize
    quat = attKin.trans_AngVecToQuat(ang, vec)
    eps = 1e-10
    isQuatScaOk = (abs(np.cos(ang/2) - quat.sca) < eps)
    isQuatVecOk = (np.max(abs(np.sin(ang/2)*vec - quat.vec)) < eps)
    isOk = isQuatScaOk and isQuatVecOk
    return isOk

def validate_trans_EulerAngToQuat():
    return "TBW"

def validate_trans_trans_DcmToEulerAng():
    return "TBW"

def validate_trans_DcmToQuat():
    return "TBW"

def validate_trans_EulerAngToDcm():
    return "TBW"

def validate_Rx():
    ang = (2*np.random.rand()-1) * deg2rad
    R = attKin.Rx(ang)
    Rprod1 = np.matmul(R, np.transpose(R))
    Rprod2 = np.matmul(np.transpose(R), R)
    eps = 1e-10
    isInvOk = (np.max(abs(Rprod1 - np.eye(3))) < eps) and (np.max(abs(Rprod2 - np.eye(3))) < eps)
    isAxOk = (R[0,0]==1 and R[1,1] == np.cos(ang) and R[1,2] == np.sin(ang))
    isOk = isInvOk and isAxOk
    return isOk

def validate_Ry():
    ang = (2*np.random.rand()-1) * deg2rad
    R = attKin.Ry(ang)
    Rprod1 = np.matmul(R, np.transpose(R))
    Rprod2 = np.matmul(np.transpose(R), R)
    eps = 1e-10
    isInvOk = (np.max(abs(Rprod1 - np.eye(3))) < eps) and (np.max(abs(Rprod2 - np.eye(3))) < eps)
    isAxOk = (R[1,1]==1 and R[0,0] == np.cos(ang) and R[0,2] == -np.sin(ang))
    isOk = isInvOk and isAxOk
    return isOk

def validate_Rz():
    ang = (2*np.random.rand()-1) * deg2rad
    R = attKin.Rz(ang)
    Rprod1 = np.matmul(R, np.transpose(R))
    Rprod2 = np.matmul(np.transpose(R), R)
    eps = 1e-10
    isInvOk = (np.max(abs(Rprod1 - np.eye(3))) < eps) and (np.max(abs(Rprod2 - np.eye(3))) < eps)
    isAxOk = (R[2,2]==1 and R[0,0] == np.cos(ang) and R[0,1] == np.sin(ang))
    isOk = isInvOk and isAxOk
    return isOk

def validate_attitudeKinematics_functions():
    print("Validate attitudeKinematics functions...")
    print("   validate_multiplyQuat:              " + str(validate_multiplyQuat()))
    print("   validate_applyRotation:             " + str(validate_applyRotation()))
    print("   validate_trans_VecRotToQuat:        " + str(validate_trans_VecRotToQuat()))
    print("   validate_trans_VecToQuat:           " + str(validate_trans_VecToQuat()))
    print("   validate_trans_VecToCrossMat:       " + str(validate_trans_VecToCrossMat()))
    print("   validate_trans_AngVecToQuat:        " + str(validate_trans_AngVecToQuat()))
    print("   validate_trans_EulerAngToQuat:      " + str(validate_trans_EulerAngToQuat()))
    print("   validate_trans_trans_DcmToEulerAng: " + str(validate_trans_trans_DcmToEulerAng()))
    print("   validate_trans_DcmToQuat:           " + str(validate_trans_DcmToQuat()))
    print("   validate_trans_EulerAngToDcm:       " + str(validate_trans_EulerAngToDcm()))
    print("   validate_Rx:                        " + str(validate_Rx()))
    print("   validate_Ry:                        " + str(validate_Ry()))
    print("   validate_Rz:                        " + str(validate_Rz()))
    print("Done")


# --------------------------------------------------
# RUN VALIDATION
# --------------------------------------------------
validate_attitudeKinematics_classes()
validate_attitudeKinematics_functions()

eulerAng = np.array([10, 10, 10])*deg2rad
Mx = attKin.Rx(eulerAng[0])
My = attKin.Ry(eulerAng[1])
Mz = attKin.Rz(eulerAng[2])

qRotX = attKin.trans_DcmToQuat(Mx)
qRotY = attKin.trans_DcmToQuat(My)
qRotZ = attKin.trans_DcmToQuat(Mz)

# M = Mx
M_ba = Mz
M_cb = My
M_ca = np.matmul(M_cb, M_ba)
# M = np.matmul(Mx, np.matmul(My, Mz))

q_ba = attKin.trans_DcmToQuat(M_ba)
q_cb = attKin.trans_DcmToQuat(M_cb)

vec_a = np.array([1, 0, 0])
vec_a = vec_a/np.linalg.norm(vec_a)
vec_b = np.matmul(M_ba, vec_a)
vec_c = np.matmul(M_ca, vec_a)

q_caTest = attKin.multiplyQuat(q_ba, q_cb)

# print(attKin.applyRotation(q_ba.conjugate(), vec_a) - vec_b)
# print(attKin.applyRotation(q_cb.conjugate(), vec_b) - vec_c)
# print(attKin.applyRotation(q_caTest.conjugate(), vec_a) - vec_c)



# qRot = attKin.multiplyQuat(qRotY, qRotX)
 # attKin.multiplyQuat(qRotX, attKin.multiplyQuat(qRotZ, qRotY))
# print(qRot.toVec()-attKin.trans_DcmToQuat(M).toVec())
# print(qRot.toEuler()/deg2rad)
# print(attKin.trans_DcmToQuat(M).toEuler()/deg2rad)
# print(attKin.trans_DcmToEulerAng(qRot.toDcm())/deg2rad)
# print(attKin.trans_DcmToEulerAng(M)/deg2rad)



# q1 = attKin.Quaternion(1, np.array([2, 3, 4]))
# q2 = attKin.Quaternion(9, np.array([8, 7, 6]))
# q3 = attKin.multiplyQuat(q1, q2)
# q4 = attKin.multiplyQuat(q2, q1)
# print(q3.sca)
# print(q3.vec)
# print(q4.sca)
# print(q4.vec)









