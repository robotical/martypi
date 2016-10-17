import pythonRobotTest as marty
import random, time

# initialise the robot
marty.hello()

for i in range(5):
	marty.walk(1, random.randint(0,50), random.randint(-20,20), 15)
	time.sleep(1.5)



# get Archie back to a standing position
marty.standStraight()

