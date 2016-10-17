import pythonRobotTest as marty

# initialise the robot
marty.hello()

# --- Write your code here

marty.leanLeft(65)
marty.liftRightLeg(50)
marty.moveRightLegForward(20)
marty.moveLeftLegBackward(20)
marty.lowerLeg()
marty.leanLeft(0)

marty.leanRight(65)
marty.liftLeftLeg(50)
marty.moveLeftLegForward(20)
marty.moveRightLegBackward(20)
marty.lowerLeg()
marty.leanRight(0)

# get Archie back to a standing position
marty.standStraight()
