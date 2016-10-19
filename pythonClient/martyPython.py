import socket
# import sys
# from time import sleep

# Robot Network Info  # TODO: Move to Config file
DEF_HOST = 'localhost'
DEF_PORT = 1569

# Command definitions, to be sent over network
cmds = {
  "hello": 0,
  "moveknee:": 1,
  "movehip": 2,
  "leansagittal": 3,
  "leansideways": 4,
  "step": 5,
  "walk": 6,
  "eyes": 7,
  "kick": 8,
  "movejoints": 9,
  "liftleg": 10,
  "lowerleg": 11,
  "celebrate": 12,
  "hiptobesquare": 13,
  "rollerskate": 14,
  "arms": 15,
}

encoding = {
  "left": 0,
  "right": 1,
  "forward": 0,
  "backward": 1,
  "positive": 0,
  "negative": 1
}

def init_socket(host, port=DEF_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (host, port)
    print "Connecting to %s port: %s" % server_address
    sock.connect(server_address)
    return sock

def sendCmdSock(sock, cmd, data):
    dsendlist = [cmd] + data
    dsend = bytearray(dsendlist)
    sock.sendall(dsend)
    sock.close()

def sendCmd(cmd, data, host=DEF_HOST):
    sock = init_socket(host, DEF_PORT)
    sendCmdSock(sock, cmd, data)

def leanSagittal(amount):
    if amount < 0:
        direction = encoding["backward"]
    else:
        direction = encoding["forward"]
    amount = abs(amount)
    if amount > 100:
        amount = 100
    sendCmd(cmds["leansagittal"], [direction, amount])

def leanForward(amount, time=20):
    if type(amount) != int and type(amount) != float:
        print "Error: leanForward needs to be sent a number"
        return False
    amount = min(abs(int(amount)), 100)
    sendCmd(cmds["leansagittal"], [encoding["forward"], amount, time])
    return True

def leanBackward(amount):
    if type(amount) != int and type(amount) != float:
        print "Error: leanBackward needs to be sent a number"
        return False
    amount = min(abs(int(amount)), 100)
    sendCmd(cmds["leansagittal"], [encoding["backward"], amount])
    return True

def leanLeft(amount):
    if type(amount) != int and type(amount) != float:
        print "Error: leanLeft needs to be sent a number"
        return False
    amount = min(abs(int(amount)), 100)
    sendCmd(cmds["leansideways"], [encoding["left"], amount])
    return True

def leanRight(amount):
    if type(amount) != int and type(amount) != float:
        print "Error: leanRight needs to be sent a number"
        return False
    amount = min(abs(int(amount)), 100)
    sendCmd(cmds["leansideways"], [encoding["right"], amount])
    return True

def liftLeg(leg, amount):
    if type(amount) != int and type(amount) != float:
        print "Error: liftLeg needs to be sent a number"
        return False
    if leg == "left":
        leg = encoding["left"]
    elif leg == "right":
        leg = encoding["right"]
    if leg != encoding["left"] and leg != encoding["right"]:
        print "Error: liftleg needs to be send a valid leg"
        return False
    amount = min(abs(int(amount)), 100)
    sendCmd(cmds["liftleg"], [leg, amount])

def liftLeftLeg(amount):
    return liftLeg(encoding["left"], amount)

def liftRightLeg(amount):
    return liftLeg(encoding["right"], amount)

def moveHip(leg, amount, movetime=20):
    if type(amount) != int and type(amount) != float:
        print "Error: moveHip needs to be sent a number"
        return False
    if leg == "left":
        leg = encoding["left"]
    elif leg == "right":
        leg = encoding["right"]
    if leg != encoding["left"] and leg != encoding["right"]:
        print "Error: liftleg needs to be send a valid leg"
        return False
    if (amount < 0):
        sign = encoding["negative"]
    else:
        sign = encoding["positive"]
    amount = min(abs(int(amount)), 100)
    sendCmd(cmds["movehip"], [leg, sign, amount, movetime])
    return True

def moveLeftHip(amount):
    return moveHip(encoding["left"], amount)

def moveRightHip(amount, movetime=20):
    return moveHip(encoding["right"], amount, movetime)

def lowerLeg(movetime = 10):
    return sendCmd(cmds["lowerleg"], [movetime])

def walk(numsteps, stepLength, turn, movetime=20):
    if stepLength > 0:
        stepDir = encoding["forward"]
    else:
        stepDir = encoding["backward"]
    if turn > 0:
        turnDir = encoding["right"]
    else:
        turnDir = encoding["left"]
    stepLength = abs(stepLength)
    turn = abs(turn)
    sendCmd(cmds["walk"], [numsteps, stepDir, stepLength, turnDir, turn, movetime])

def eyes(amount, amount2=None):
    if (amount < 0):
        sign = encoding["negative"]
    else:
        sign = encoding["positive"]
    amount = min(abs(int(amount)), 100)
    cmd = [sign, amount]
    if amount2 != None:
        if (amount2 < 0):
            sign = encoding["negative"]
        else:
            sign = encoding["positive"]
        amount2 = min(abs(int(amount2)), 100)
        cmd = cmd + [sign, amount2]
    sendCmd(cmds["eyes"], cmd)

def arms(angle1, angle2):
    sendCmd(cmds["arms"], [angle1, angle2])

def hello():
    sendCmd(cmds["hello"], [])

def moveRightLegForward(amount, movetime=10):
    return moveRightHip(0-abs(amount), movetime)

def moveRightLegBackward(amount):
    return moveRightHip(abs(amount))

def moveLeftLegForward(amount):
    return moveLeftHip(0-abs(amount))

def moveLeftLegBackward(amount):
    return moveLeftHip(abs(amount))

def standStraight():
    leanForward(0)
    return leanLeft(0)

def kickLeft():
    return sendCmd(cmds["kick"], [encoding["left"]])

def kickRight():
    return sendCmd(cmds["kick"], [encoding["right"]])

def celebrate():
    return sendCmd(cmds["celebrate"],[])

def hipToBeSquare():
    sendCmd(cmds["hiptobesquare"], [1], "martyZero")
    sendCmd(cmds["hiptobesquare"], [0], "marty2")
    return sendCmd(cmds["hiptobesquare"], [2], "marty4")

def rollerSkate():
    sendCmd(cmds["rollerskate"], [])
