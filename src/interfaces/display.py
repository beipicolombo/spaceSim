import numpy as np
from vpython import *

import src.attitudeKinematics as attitudeKinematics

# To be moved as common constants
pi  = np.pi
deg2rad = pi/180


def getVecForGraphics(myArray):
    return vec(myArray[0], myArray[1], myArray[2])


class Displays():
    def __init__(self):
        self.bodyDisplay = {}
        self.fswDisplay = {}

    def initialize(self, simParam, modelsBus, fswBus, fswModeMgtState):
        # Set the display if required
        if simParam.runOptions.swVisualPy:

            # 1. Spacecraft and reference frames
            # Use dummy S/C dimensions => geometry parameters to be encapsulated
            lx = 1 # [m]
            ly = 0.5 # [m]
            lz = 0.5 # [m]
            # 1.1 Inertial frame axes
            arrow_xI_I = arrow(pos=vec(0, 0, 0), axis=vec(1, 0, 0), color=color.red, shaftwidth  = 0.01)
            arrow_yI_I = arrow(pos=vec(0, 0, 0), axis=vec(0, 1, 0), color=color.red, shaftwidth  = 0.01)
            arrow_zI_I = arrow(pos=vec(0, 0, 0), axis=vec(0, 0, 1), color=color.red, shaftwidth  = 0.01)
            label_xI_I = label(pos=vec(1, 0, 0), text = "xI", color = color.red, xoffset = 10, yoffset = 10, box = False, line = False)
            label_yI_I = label(pos=vec(0, 1, 0), text = "yI", color = color.red, xoffset = 10, yoffset = 10, box = False, line = False)
            label_zI_I = label(pos=vec(0, 0, 1), text = "zI", color = color.red, xoffset = 10, yoffset = 10, box = False, line = False)
            self.bodyDisplay.update({"arrow_xI_I": arrow_xI_I})
            self.bodyDisplay.update({"arrow_yI_I": arrow_yI_I})
            self.bodyDisplay.update({"arrow_zI_I": arrow_zI_I})
            self.bodyDisplay.update({"label_xI_I": label_xI_I})
            self.bodyDisplay.update({"label_yI_I": label_yI_I})
            self.bodyDisplay.update({"label_zI_I": label_zI_I})

            # 1.2 Body frame axes
            qBI_sca = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].value
            qBI_vec = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].value
            dcmIB = np.transpose(attitudeKinematics.Quaternion(qBI_sca, qBI_vec).toDcm())
            arrow_xB_I = arrow(pos=vec(0, 0, 0), axis=getVecForGraphics(dcmIB[:,0]), color=color.blue, shaftwidth  = 0.01)
            arrow_yB_I = arrow(pos=vec(0, 0, 0), axis=getVecForGraphics(dcmIB[:,1]), color=color.blue, shaftwidth  = 0.01)
            arrow_zB_I = arrow(pos=vec(0, 0, 0), axis=getVecForGraphics(dcmIB[:,2]), color=color.blue, shaftwidth  = 0.01)
            self.bodyDisplay.update({"arrow_xB_I": arrow_xB_I})
            self.bodyDisplay.update({"arrow_yB_I": arrow_yB_I})
            self.bodyDisplay.update({"arrow_zB_I": arrow_zB_I})

            # 1.3 Set Nadir frame axes
            qLI_sca = modelsBus.subBuses["environment"].signals["qLI_sca"].value
            qLI_vec = modelsBus.subBuses["environment"].signals["qLI_vec"].value
            dcmIL = np.transpose(attitudeKinematics.Quaternion(qLI_sca, qLI_vec).toDcm())
            arrow_xL_I = arrow(pos=vec(0, 0, 0), axis=getVecForGraphics(dcmIL[:,0]), color=color.green, shaftwidth  = 0.01)
            arrow_yL_I = arrow(pos=vec(0, 0, 0), axis=getVecForGraphics(dcmIL[:,1]), color=color.green, shaftwidth  = 0.01)
            arrow_zL_I = arrow(pos=vec(0, 0, 0), axis=getVecForGraphics(dcmIL[:,2]), color=color.green, shaftwidth  = 0.01)
            label_xL_I = label(pos=getVecForGraphics(dcmIL[:,0]), text = "xL", color = color.green, xoffset = 10, yoffset = 10, box = False, line = False)
            label_yL_I = label(pos=getVecForGraphics(dcmIL[:,1]), text = "yL", color = color.green, xoffset = 10, yoffset = 10, box = False, line = False)
            label_zL_I = label(pos=getVecForGraphics(dcmIL[:,2]), text = "zL", color = color.green, xoffset = 10, yoffset = 10, box = False, line = False)
            self.bodyDisplay.update({"arrow_xL_I": arrow_xL_I})
            self.bodyDisplay.update({"arrow_yL_I": arrow_yL_I})
            self.bodyDisplay.update({"arrow_zL_I": arrow_zL_I})
            self.bodyDisplay.update({"label_xL_I": label_xL_I})
            self.bodyDisplay.update({"label_yL_I": label_yL_I})
            self.bodyDisplay.update({"label_zL_I": label_zL_I})  

            # 1.4 SC
            SC = box(pos=vec(0, 0, 0), axis = getVecForGraphics(dcmIB[:,0]), length=lx, height=ly, width=lz, color=color.cyan)
            SC.up = getVecForGraphics(dcmIB[:,1])
            self.bodyDisplay.update({"SC": SC})
            
            # 2. FSW display
            # FSW label parameters
            labelsHeight = 15
            labelsYoffset = 140
            labelsYoffsetDelta = 20

            # 2.1 AOCS mode
            label_aocsMode = label(pos = vec(0, 0, 0), text = ("MODE: " + fswModeMgtState.aocsMode), xoffset = 150, yoffset = 160, height = labelsHeight, line = False, box = False)
            self.fswDisplay.update({"label_aocsMode": label_aocsMode})
            
            # 2.2 Angular rates wrt inertial frame
            angRate_BI_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].value
            label_angRate_BI_B_X = label(pos = vec(0, 0, 0), text = ("RATE (X): " + ("%.2f" % (angRate_BI_B[0]/deg2rad)) + " deg/s"), xoffset = 150, yoffset = labelsYoffset, height = 10, line = False, box = False)
            labelsYoffset = labelsYoffset - labelsYoffsetDelta
            label_angRate_BI_B_Y = label(pos = vec(0, 0, 0), text = ("RATE (Y): " + ("%.2f" % (angRate_BI_B[1]/deg2rad)) + " deg/s"), xoffset = 150, yoffset = labelsYoffset, height = 10, line = False, box = False)
            labelsYoffset = labelsYoffset - labelsYoffsetDelta
            label_angRate_BI_B_Z = label(pos = vec(0, 0, 0), text = ("RATE (Z): " + ("%.2f" % (angRate_BI_B[2]/deg2rad)) + " deg/s"), xoffset = 150, yoffset = labelsYoffset, height = 10, line = False, box = False)
            labelsYoffset = labelsYoffset - labelsYoffsetDelta
            self.fswDisplay.update({"label_angRate_BI_B_X": label_angRate_BI_B_X})
            self.fswDisplay.update({"label_angRate_BI_B_Y": label_angRate_BI_B_Y})
            self.fswDisplay.update({"label_angRate_BI_B_Z": label_angRate_BI_B_Z})

            # 2.3 Attitude wrt inertial frame
            eulerAng_BI = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].value
            label_eulerAng_BI_R = label(pos = vec(0, 0, 0), text = ("ROLL (X): " + ("%.2f" % (eulerAng_BI[0]/deg2rad)) + " deg"), xoffset = 150, yoffset = labelsYoffset, height = 10, line = False, box = False)
            labelsYoffset = labelsYoffset - labelsYoffsetDelta
            label_eulerAng_BI_P = label(pos = vec(0, 0, 0), text = ("PITCH (Y): " + ("%.2f" % (eulerAng_BI[1]/deg2rad)) + " deg"), xoffset = 150, yoffset = labelsYoffset, height = 10, line = False, box = False)
            labelsYoffset = labelsYoffset - labelsYoffsetDelta
            label_eulerAng_BI_Y = label(pos = vec(0, 0, 0), text = ("YAW (Z): " + ("%.2f" % (eulerAng_BI[2]/deg2rad)) + " deg"), xoffset = 150, yoffset = labelsYoffset, height = 10, line = False, box = False)
            self.fswDisplay.update({"label_eulerAng_BI_R": label_eulerAng_BI_R})
            self.fswDisplay.update({"label_eulerAng_BI_P": label_eulerAng_BI_P})
            self.fswDisplay.update({"label_eulerAng_BI_Y": label_eulerAng_BI_Y})

    def update(self, simParam, modelsBus, fswBus, fswModeMgtState, qPrevLI, qPrevBI):
        if simParam.runOptions.swVisualPy:
            # Set rate
            rate(simParam.runOptions.visualPyRate)
            
            # Pre-process attitude information from buses
            qBI_sca = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_sca"].value
            qBI_vec = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["qBI_vec"].value
            qBI = attitudeKinematics.Quaternion(qBI_sca, qBI_vec)
            dcmIB = np.transpose(attitudeKinematics.Quaternion(qBI_sca, qBI_vec).toDcm())
            
            qLI_sca =  modelsBus.subBuses["environment"].signals["qLI_sca"].value
            qLI_vec =  modelsBus.subBuses["environment"].signals["qLI_vec"].value
            qLI = attitudeKinematics.Quaternion(qLI_sca, qLI_vec)
            dcmIL = np.transpose(attitudeKinematics.Quaternion(qLI_sca, qLI_vec).toDcm())
            
            qBBprev = attitudeKinematics.multiplyQuat(qPrevBI.conjugate(), qBI).normalize()
            qLLprev = attitudeKinematics.multiplyQuat(qPrevLI.conjugate(), qLI).normalize()
            (rotDirVec_BBprev, rotAng_BBprev) = qBBprev.toVecRot()
            (rotDirVec_LLprev, rotAng_LLprev) = qLLprev.toVecRot()
            
            # 1. Spacecraft and reference frames
            # 1.1 Body frame axes
            self.bodyDisplay["arrow_xB_I"].axis =  getVecForGraphics(dcmIB[:,0])
            self.bodyDisplay["arrow_yB_I"].axis =  getVecForGraphics(dcmIB[:,1])
            self.bodyDisplay["arrow_zB_I"].axis =  getVecForGraphics(dcmIB[:,2])
            # 1.2 Nadir frame axes
            self.bodyDisplay["arrow_xL_I"].axis =  getVecForGraphics(dcmIL[:,0])
            self.bodyDisplay["arrow_yL_I"].axis =  getVecForGraphics(dcmIL[:,1])
            self.bodyDisplay["arrow_zL_I"].axis =  getVecForGraphics(dcmIL[:,2])
            # 1.3 SC
            self.bodyDisplay["SC"].axis = getVecForGraphics(dcmIB[:,0])
            self.bodyDisplay["SC"].up = getVecForGraphics(dcmIB[:,1])
        
            # 2. FSW display
            # 2.1 AOCS mode
            self.fswDisplay["label_aocsMode"].text = ("MODE: " + fswModeMgtState.aocsMode)

            # 2.2 Angular rates wrt inertial frame
            angRate_BI_B = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].value
            self.fswDisplay["label_angRate_BI_B_X"].text = ("RATE (X): " + ("%.2f" % (angRate_BI_B[0]/deg2rad)) + " deg/s")
            self.fswDisplay["label_angRate_BI_B_Y"].text = ("RATE (Y): " + ("%.2f" % (angRate_BI_B[1]/deg2rad)) + " deg/s")
            self.fswDisplay["label_angRate_BI_B_Z"].text = ("RATE (Z): " + ("%.2f" % (angRate_BI_B[2]/deg2rad)) + " deg/s")
            
            # 2.3 Attitude wrt inertial frame
            eulerAng_BI = modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].value
            self.fswDisplay["label_eulerAng_BI_R"].text = ("ROLL (X): " + ("%.2f" % (eulerAng_BI[0]/deg2rad)) + " deg")
            self.fswDisplay["label_eulerAng_BI_P"].text = ("PITCH (Y): " + ("%.2f" % (eulerAng_BI[1]/deg2rad)) + " deg")
            self.fswDisplay["label_eulerAng_BI_Y"].text = ("YAW (Z): " + ("%.2f" % (eulerAng_BI[2]/deg2rad)) + " deg")