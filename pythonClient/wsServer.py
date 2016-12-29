
HOST = ''                 # Symbolic name meaning all available interfaces
PORT = 5996              # Arbitrary non-privileged port


from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
import martyPython as marty
import json
from time import sleep
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN)

class SimpleEcho(WebSocket):

    def handleMessage(self):
        # echo message back to client
        print 'recieved: ', self.data
        try:
            rcv = json.loads(self.data)
            print 'id: ', rcv["id"]
            print self.data

            #self.sendMessage(u'testing')
            if rcv["cmd"] == "celebrate":
                marty.celebrate()
                sleep(2.0)
                self.sendMessage(self.data)
            if rcv["cmd"] == "readSensor":
                rcv["sensorData"] = GPIO.input(17)
                response = json.dumps(rcv)
                self.sendMessage(unicode(response))
            if rcv["cmd"] == "walk":
                marty.walk(rcv["numsteps"], rcv["steplength"], rcv["turn"], rcv["movetime"])
            if rcv["cmd"] == "kick":
                if rcv["leg"] == "left":
                    marty.kickLeft()
                else:
                    marty.kickRight()
                sleep(1.5)
                self.sendMessage(self.data)
            if rcv["cmd"] == "hello":
                marty.hello()
            if rcv["cmd"] == "eyes":
                if rcv["eyes"] == "wide":
                    eyes = -50
                elif rcv["eyes"] == "angry":
                    eyes = 20
                else:
                    eyes = 0
                print 'eyes', eyes
                marty.eyes(eyes)
                self.sendMessage(self.data)
            if rcv["cmd"] == "lean":
                amount = max(min(int(rcv["amount"]), 60),0)
                if rcv["direction"] == "left":
                    marty.leanLeft(amount)
                else:
                    marty.leanRight(amount)
                sleep(1.0)
                self.sendMessage(self.data)
            if rcv["cmd"] == "liftLeg":
                amount = max(min(int(rcv["amount"]), 40),0)
                if rcv["leg"] == "left":
                    marty.liftLeg("left", amount)
                else:
                    marty.liftLeg("right", amount)
                sleep(1.0)
                self.sendMessage(self.data)
            if rcv["cmd"] == "lowerLeg":
                marty.lowerLeg()
                sleep(1.0)
                self.sendMessage(self.data)
            if rcv["cmd"] == "moveHip":
                amount = max(min(int(rcv["amount"]), 40),0)
                movetime = max(min(int(rcv["movetime"]), 30),5)
                if rcv["direction"] == "forward":
                    amount = -1*amount
                marty.moveHip(rcv["leg"], amount, movetime)
                sleep(float(movetime)/10)
                self.sendMessage(self.data)

        except:
            print 'no id received'

    def handleConnected(self):
        print self.address, 'connected'

    def handleClose(self):
        print self.address, 'closed'

server = SimpleWebSocketServer('', PORT, SimpleEcho)
server.serveforever()
