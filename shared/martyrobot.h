#ifndef __ARCHIE_ARCHIEROBOT
#define __ARCHIE_ARCHIEROBOT

#include <iostream>
#include <deque>
#include <stdint.h>
#include "i2c.h"
#include "martyCalibration.h"
#include "servoboardutils.h"

using namespace std;

#define DEBUG_MODE false

// Console printing MACROS
#define ERR(x) cerr << "\033[22;31;1m" << x << "\033[0m";    // RED
#define WARN(x) cerr << "\033[22;33;1m" << x << "\033[0m";   // YELLOW
#define INFO(x) cerr << "\033[22;37;1m" << x << "\033[0m";   // WHITE
#define DEBUG(x) cerr << "\033[22;34;1m" << x << "\033[0m";  // BLUE
#define CLEAR() cerr << "\x1B[2J\x1B[H"; // Clear Console

enum jointNames {
  R_RHIP,
  R_RTWIST,
  R_RKNEE,
  R_LHIP,
  R_LTWIST,
  R_LKNEE,
  R_EYES,
  R_RARM,
  R_LARM,
  R_AUX1,
  R_AUX2,
  R_NUMJOINTS // Number of Joints, Do Not Remove!
};

struct robotJoint {
  int cmdZero;
  int cmdMin;
  int cmdMax;
  int cmdDir;
  int cmdMult;
  uint8_t servoChannel;
};

class martyrobot {
 private:
  unsigned char deviceAddr;
  robotJoint joint[R_NUMJOINTS];
  uint16_t jointPosToServoCmd(float pos, uint16_t zero, float mult, short dir,
                              uint16_t max, uint16_t min);
 public:
  bool setServoJointPulse(uint8_t jointIndex, uint16_t pos);
  bool setServoPulse(uint8_t channel,
                     uint16_t pos);   // this could be private. ATM only required for calibration
  bool stopServo(uint8_t channel);
  bool stopRobot();

  int   numjoints;
  myI2C* i2cptr;
  std::deque<float> jangles;
  bool setServo(int joint, float angle);
  bool setServos(std::deque <float> angles);
  bool init(bool calibrating = false);
  void printAngles();
};

#endif
