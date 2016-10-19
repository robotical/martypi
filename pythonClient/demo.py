import time
import math
import martyPython as marty

# Initialise Marty
# marty.hello()
# time.sleep(3.0)

# Example Walk
NUM_STEPS = 2
STEP_LENGTH = 50
TURN = 0      # 0 = Straight, >0 = Right, <0 = Left
MOVE_TIME = 20
marty.walk(NUM_STEPS, STEP_LENGTH, TURN, MOVE_TIME)

# TODO: Debug rest of demos when eyes/arms implemented

# for x in range(0,20):
#     marty.eyes(-60)
#     marty.arms(100,200)
#     time.sleep(0.1)
#     marty.eyes(-40)
#     marty.arms(200,100)
#     time.sleep(0.1)
# marty.arms(200,200)

# marty.walk(NUM_STEPS,50,0,17)
# marty.walk(1,10,0,13)
# marty.arms(200,200)
# time.sleep(NUM_STEPS*1.7 + 6.0)
# marty.eyes(-60)

# marty.leanForward(50,15)
# marty.leanForward(0,15)
# marty.eyes(-80)

## Example Step procedure
# def step():
#   marty.standStraight()
#   marty.leanRight(60)
#   marty.liftLeftLeg(50)
#   marty.moveLeftLegForward(50)
#   marty.lowerLeg()
#   marty.standStraight()
#   marty.leanLeft(60)
#   marty.liftRightLeg(10)
#   marty.moveRightLegForward(50)
#   marty.lowerLeg()

# for i in range(5):
#   step()

# marty.kickRight()
# marty.celebrate()

## Get Marty back to a standing position
# marty.standStraight()
