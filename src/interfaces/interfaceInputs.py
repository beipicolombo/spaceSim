import numpy as np
import serial
import serial.tools.list_ports as port_list
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


class InterfaceInputsParam:
	def __init__(self):
		self.joystickParam = JoystickParam()

# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def getInterfaceInputs(fswBusIn, joystickSerialPort, interfaceInputsParam):
	# Get joystick inputs
	fswInterfacesBusOut = getJoystickCmdInputs(fswBusIn, joystickSerialPort, interfaceInputsParam.joystickParam)
	return fswInterfacesBusOut


def getJoystickCmdInputs(fswBusIn, joystickSerialPort, joystickParam):
	# Initialize output
	fswInterfacesBusOut = fswBusIn.subBuses["interfaces"]

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



# --------------------------------------------------
# SIMULATION / TEST
# --------------------------------------------------