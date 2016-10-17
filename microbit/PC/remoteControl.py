import sys, os
sys.path.append(os.path.abspath(os.path.join('../..', 'pythonClient')))
import pythonRobotTest as marty
import serial
import struct
import time
from math import copysign

MAXTURN = 40
MAXWALKTURN = 20

# start comms with microbit (hopefully!)
ser = serial.Serial('/dev/ttyACM0', 115200)

TURN_THRESH = 10

marty.hello()

#marty.walk(2, 100, 0)
ser.flushInput()
lastCmdTime = time.clock()
while True:
	if (ser.inWaiting() > 4):
		bFromMB = ser.read(ser.inWaiting())

		ax = struct.unpack('B', bFromMB[0])[0] - 128
		ay = struct.unpack('B', bFromMB[1])[0] - 128
		ba = struct.unpack('B', bFromMB[2])[0]
		bb = struct.unpack('B', bFromMB[3])[0]

		if abs(ax) > TURN_THRESH:
			turn = copysign(abs(ax-TURN_THRESH), ax)*2
			turn = -1*min(MAXTURN, max(0-MAXTURN, int(turn/2)))
		else:
			turn = 0

		#print ax, ay, turn

		if time.clock() - lastCmdTime > 1.2:
			if ba and bb:
				marty.celebrate()
				time.sleep(2.5)
				lastCmdTime = time.clock()
			elif ba:
				marty.kickLeft()
				time.sleep(1.5)
				lastCmdTime = time.clock()
			elif bb:
				marty.kickRight()
				time.sleep(1.5)
				lastCmdTime = time.clock()
			elif abs(ay) > 20:
				forward = min(60, max(-60, -2*ay))
				#if turn < 0:		# correct for difficulty in turning right
				#	turn = max(-15, turn*2)
				turn = min(MAXWALKTURN, max(0-MAXWALKTURN, int(turn/2)))
				lastCmdTime = time.clock()
				marty.walk(1, forward, turn, 12)

				print forward, '\t', turn
			elif turn:
				marty.walk(2, 0, turn, 13)
				time.sleep(1.6)
				lastCmdTime = time.clock()
				print '\t', turn



#marty.standStraight()
ser.close()
