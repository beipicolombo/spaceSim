import numpy as np
import serial
import serial.tools.list_ports as port_list
import keyboard
import time
import re


# --------------------------------------------------
# CLASSES
# --------------------------------------------------
class JoystickParam:
	def __init__(self):
		self.isJoystickUsed = False
		self.joystickSerialPortName = ""
		
		self.x0Calibr = 2775
		self.y0Calibr = 2802

	def initializeSerialPort(self):
		if self.isJoystickUsed:
			joystickSerialPort = serial.Serial(self.joystickSerialPortName)
		else:
			joystickSerialPort = serial.Serial()
		return joystickSerialPort


class KeyboardInputsParam:
	def __init__(self):
		self.isKeyboardUsed = False
		self.trqAmplitude_X = 0.01 # [N*m]
		self.trqAmplitude_Y = 0.01 # [N*m]


class Param:
	def __init__(self):
		self.joystickParam = JoystickParam()
		self.keyboardInputsParam = KeyboardInputsParam()

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def getInterfaceInputs(fswBusIn, simBus, joystickSerialPort, interfaceInputsParam):
	fswInterfacesBusIn = fswBusIn.subBuses["interfaces"]
	# Get joystick inputs
	fswInterfacesBusOut = getJoystickCmdInputs(fswInterfacesBusIn, joystickSerialPort, interfaceInputsParam.joystickParam)
	# Get Keyboard inputs
	fswInterfacesBusOut = getKeyboardCmdInputs(fswInterfacesBusOut, simBus, interfaceInputsParam.keyboardInputsParam)
	return fswInterfacesBusOut


def getKeyboardCmdInputs(fswInterfacesBusIn, simBus, keyboardInputsParam):
	# Initialize output
	fswInterfacesBusOut = fswInterfacesBusIn

	if keyboardInputsParam.isKeyboardUsed:

		# print((keyboard.read_key() + " " + str(simBus.signals["elapsedTime"].value)))
		trqCmdManual_B = mapKeyboardKeysToTorque(keyboardInputsParam)
		# trqCmdManual_B = np.array([0, 0, 0])
	else:
		trqCmdManual_B = np.array([0, 0, 0])

    # Set output signal values
	fswInterfacesBusOut.signals["trqCmdManual_B"].update(trqCmdManual_B)
	return fswInterfacesBusOut


def getJoystickCmdInputs(fswInterfacesBusIn, joystickSerialPort, joystickParam):
	# Initialize output
	fswInterfacesBusOut = fswInterfacesBusIn

	if joystickParam.isJoystickUsed:
		# Retrieve useful signals and parameters
		x0 = joystickParam.x0Calibr
		y0 = joystickParam.y0Calibr
		
		# Process
		filterData = lambda dataStr : re.findall(r'\b\d+\b', dataStr)
		# Raw data
		dataRaw = serialPort.readline()
		dataIn = str(dataRaw, 'ascii').split(",")
		(xu, yu) = (float(filterData(dataIn[0])[0]), float(filterData(dataIn[1])[0]))
		# Build  vector
		myVec = np.array([(xu-x0), (yu-y0), 4095])
		myVec = myVec / np.linalg.norm(myVec)
		# Compute the resulting command torque in body frame
		trqCmdManual_B = np.cross(myVec, np.array([0, 0, 1]))
	else:
		trqCmdManual_B = np.array([0, 0, 0])

    # Set output signal values
	fswInterfacesBusOut.signals["trqCmdManual_B"].update(trqCmdManual_B)
	return fswInterfacesBusOut


def mapKeyboardKeysToTorque(keyboardInputsParam):
	# Initialize outputs
	trq_z = 0 

	# Roll comma
	if keyboard.is_pressed("droite"):
		trq_x = +keyboardInputsParam.trqAmplitude_X
	elif keyboard.is_pressed("gauche"):
		trq_x = -keyboardInputsParam.trqAmplitude_X
	else:
		trq_x = 0

	# Pitch command
	if keyboard.is_pressed("haut"):
		trq_y = -keyboardInputsParam.trqAmplitude_Y
	elif keyboard.is_pressed("bas"):
		trq_y = +keyboardInputsParam.trqAmplitude_Y
	else:
		trq_y = 0

	trqCmdManual_B = np.array([trq_x, trq_y, trq_z])
	time.sleep(0.01)
	return trqCmdManual_B

# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------