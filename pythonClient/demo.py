import pythonRobotTest as marty
import time
import math

# initialise the robot
#marty.hello()
#time.sleep(3.0)

marty.walk(2, 50, 0, 16)
marty.kettle()

numsteps = 3
# --- Write your code
'''
for x in range(0,20):
	marty.eyes(-60)
	marty.arms(100,200)
	time.sleep(0.1)
	marty.eyes(-40)
	marty.arms(200,100)
	time.sleep(0.1)

marty.arms(200,200)
'''
'''
marty.walk(numsteps,50,0,17)
marty.walk(1,10,0,13)
marty.arms(200,200)
time.sleep(numsteps*1.7 + 6.0)
marty.eyes(-60)
'''
#marty.leanForward(50,15)
#marty.leanForward(0,15)
#marty.eyes(-80)

# --- Some code as finished by this point

'''
def step():
	marty.standStraight()
	marty.leanRight(60)
	marty.liftLeftLeg(50)
	marty.moveLeftLegForward(50)
	marty.lowerLeg()
	marty.standStraight()
	marty.leanLeft(60)
	marty.liftRightLeg(10)
	marty.moveRightLegForward(50)
	marty.lowerLeg()

for i in range(5):
	step()

#marty.walk(20, 50, 0, 12)
'''

'''
marty.kickRight()
marty.celebrate()

# get Archie back to a standing position
marty.standStraight()
'''
