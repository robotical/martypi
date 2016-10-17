# original code from 

# This is required to be able to "Monkey Patch"  (https://www.google.com/search?q=define:monkey+patch) a number of 
# modules in the Python standard library to make them cooperative. This gives us an easy way to play with the idea
# of asynchronous programming.
import gevent

# The following import will allow us to view exceptions with a good level of detail in the case of something unexpected.
import sys, traceback
from time import sleep
# The configuration class is what we will use to read from our special configuartion file.
#from configuration import Configuration

# import python client for Marty
import sys, os
sys.path.append(os.path.abspath(os.path.join('..', 'pythonClient')))
import pythonRobotTest as marty

# This import will give us our wrapper for the Pocketsphinx library which we can use to get the voice commands from the 
# user.
from pocket_sphinx_listener import PocketSphinxListener

#gevent.monkey.patch_all()

nums = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5}
STEPLENGTH = 50
TURNAMOUNT = 30
ARMANGLE = 200      # when raised

def runMain():
    # First, we import our devices from our configuration file. These will be split into two different groups, those
    # controlled by Philips Hue and those controlled by Insteon.
#    configuration = Configuration()
#    config = configuration.loadConfig()

    # Now we set up the voice recognition using Pocketsphinx from CMU Sphinx.
    pocketSphinxListener = PocketSphinxListener()

    # We want to run forever, or until the user presses control-c, whichever comes first.
    while True:
        try:
            command = pocketSphinxListener.getCommand().lower()

            # can just ignore the word marty in the sentence. similarly for 'your'
            command = command.replace('marty', '')
            command = command.replace('your', '')
            cwords = command.split()

            if command.startswith('walk') or command.startswith('turn'):
                direction = cwords[1]
                if len(cwords) > 2:
                    numsteps = nums[cwords[2]]
                else:
                    numsteps = 1
                if cwords[0] == 'walk':
                    if direction.startswith('forward'):
                        distance = STEPLENGTH
                    else:
                        distance = 0-STEPLENGTH
                    turn = 0
                else:
                    if direction == 'left':
                        turn = TURNAMOUNT
                    else:
                        turn = 0-TURNAMOUNT
                    distance = 0
                marty.walk(numsteps, distance, turn, 15)
                sleep((numsteps * 1.5)+0.2)
            elif cwords[0] == 'kick':
                if cwords[1] == 'left':
                    marty.kickLeft()
                else:
                    marty.kickRight()
                sleep(2)
            elif cwords[0] == 'dance':
                marty.celebrate()
                sleep(4)
            elif cwords[0] == 'hello':
                marty.hello()
                sleep(1.0)
                for i in range(3):
                    marty.arms(200,0)
                    sleep(0.5)
                    marty.arms(75,0)
                    sleep(0.5)
                marty.arms(0,0)
                sleep(2)
            elif cwords[2] == 'arm':
                if cwords[0] == 'raise':
                    armangle = ARMANGLE
                else:
                    armangle = 0
                # this is not ideal as it'll put the other arm down when one raises. and in practice will lower both arms...
                if cwords[1] == 'left':
                    leftangle = ARMANGLE
                    rightangle = 0
                else:
                    leftangle = 0
                    rightangle = 0-ARMANGLE
                marty.arms(leftangle, rightangle);

        # This will allow us to be good cooperators and sleep for a second.
        # This will give the other greenlets which we have created for talking 
        # to the Hue and Insteon hubs a chance to run.
            gevent.sleep(1)

        except (KeyboardInterrupt, SystemExit):
            print 'People sometimes make mistakes, Goodbye.'
            sys.exit()
        except Exception as e:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback,
                                      limit=2,
                                      file=sys.stdout)
            command = ''
            #sys.exit()


if __name__ == '__main__':
    runMain()
