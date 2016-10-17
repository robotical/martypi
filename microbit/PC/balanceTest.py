import sys, os
sys.path.append(os.path.abspath(os.path.join('../..', 'pythonClient')))
import pythonRobotTest as marty
import serial
import struct
import time

# start comms with microbit (hopefully!)
ser = serial.Serial('/dev/ttyACM0', 115200)

LEAN_THRESHOLD = 12
Z_THRESH = 61

marty.hello()

#marty.walk(2, 100, 0)
ser.flushInput()
lastCmdTime = time.clock()
keepGoing = True
legUp = False

time.sleep(1.0)
while (ser.inWaiting()):
	ser.read(ser.inWaiting())

filtersize = 4
axH = filtersize*[0.0]
axHi = 0
filteredAX = 0.0

prevAX = 0

while keepGoing == True:
	if (ser.inWaiting() > 4):
		bFromMB = ser.read(ser.inWaiting())

		ax = struct.unpack('B', bFromMB[0])[0] - 128
		ay = struct.unpack('B', bFromMB[1])[0] - 128
		ba = struct.unpack('B', bFromMB[2])[0]
		bb = struct.unpack('B', bFromMB[3])[0]
		az = struct.unpack('B', bFromMB[4])[0] - 128

		if not legUp and ba:
			marty.leanLeft(45)
			marty.liftRightLeg(90)
			marty.moveRightLegForward(90, 5)
			legUp = True

		if bb:
			keepGoing = False

		filteredAX = filteredAX - (float(axH[axHi]) - ax)/filtersize
		axH[axHi] = ax
		axHi = (axHi+1)%filtersize


		dax = ax-prevAX
		prevAX = ax

		print ax, '\t', dax, '\t', ay, '\t', az

		if legUp and -1*ax >= LEAN_THRESHOLD:
		#if legUp and abs(az) < Z_THRESH:
			marty.lowerLeg(2)
			marty.moveRightHip(0,2)
			marty.standStraight()
			time.sleep(1.5)
			legUp = False


marty.standStraight()
ser.close()
