import sys, os
#sys.path.append(os.path.abspath(os.path.join('../..', 'pythonClient')))
sys.path.append(os.path.abspath(os.path.join('../pythonClient')))
import pythonRobotTest as marty
import pty, serial
import struct
import time

# create virtual serial port for scratch to attach to
master, slave = pty.openpty()
s_name = os.ttyname(slave)

print 'Serial port opened at ', s_name

