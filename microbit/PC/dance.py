import sys, os
sys.path.append(os.path.abspath(os.path.join('../..', 'pythonClient')))
import pythonRobotTest as marty
import serial
import struct
import time
from math import copysign

# start comms with microbit (hopefully!)
#ser = serial.Serial('/dev/ttyACM0', 115200)


marty.celebrate()
#marty.hello()
