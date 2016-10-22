# MartyPi API
Pre-release code for calibrating and controlling Marty the Robot using a RaspberryPi from Robotical Ltd.

## PRE-RELEASE CODE
Please be aware that this code is under development, and support may be sporadic and eventually discontinued. However, code improvements are welcome, and pull-requests will be reviewed and merged if beneficial. Please open up issues on the GitHub repository detailing any fixes or suggested features giving enough information for their implementation.

## GENERAL NOTES
Before executing code, please run `make` inside the relevant software folder (e.g. `martypi/calibration`), and then run with `./<executable>` (e.g. `./calibration`). Whenever the code is changed, including after Marty's joints are calibrated, the code needs to be re-compiled before the edits take effect.

## EXECUTABLES
### - Calibration
This software provides the ability to calibrate Marty's joints to their home positions. A keyboard interface is provided to control each of the officially supported Servo joints, with the ability to disable Servos so the joints may be moved by hand. Calibration is required before any other activities may be executed safely. **Please remember to re-compile your code afterwards in order to use the new home positions.**

### - Demo
A small demo, showing most of Marty's movement capabilities, a good way to test your Marty is in tip top shape!

### - CmdServer / PythonClient
The CmdServer provides an interface to call any of Marty's moves remotely (e.g. via a Python Interface), and example PythonClient is included.

### - Dance / hipToBeSquare
Multiple example dances, feel free to expand and improvise ;)

### - VoiceControl / microbit
UNDER DEVELOPMENT
