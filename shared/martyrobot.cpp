#include "martyrobot.h"

uint16_t martyrobot::jointPosToServoCmd(float pos, uint16_t zero, float mult,
                                        short dir, uint16_t max, uint16_t min) {
  if (DEBUG_MODE) {
    printf("posToCmd: pos: %.2f, zero: %d, mult: %.2f, dir: %d, max: %d, min: %d\n",
           pos, zero, mult, dir, max, min);
  }
  uint16_t cmd;
  cmd = zero + pos * mult * dir;
  if (cmd < min) {
    cmd = min;
  } else if (cmd > max) {
    cmd = max;
  }
  return cmd;
}

bool martyrobot::setServoJointPulse(uint8_t jointIndex, uint16_t pos) {
  setPWM(this->i2cptr, this->deviceAddr, this->joint[jointIndex].servoChannel,
         pos);
  return true;
}

bool martyrobot::setServoPulse(uint8_t channel, uint16_t pos) {
  setPWM(this->i2cptr, this->deviceAddr, channel, pos);
  return true;
}

bool martyrobot::stopServo(uint8_t jointIndex) {
  setPWM(this->i2cptr, this->deviceAddr, this->joint[jointIndex].servoChannel, 0);
  return true;
}

bool martyrobot::stopRobot() {
  for (int ch = 0; ch < CH_NUM; ++ch) {
    setPWM(this->i2cptr, this->deviceAddr, ch, 0);
  }
  return true;
}

bool martyrobot::init(bool calibrating) {
  if (!CALIBRATED && !calibrating) {
    ERR("MARTY IS NOT CALIBRATED, PLEASE CALIBRATE FIRST\n");
    exit(0);
  }

  this->deviceAddr = I2CADDRESS;
  this->numjoints = R_NUMJOINTS;

  for (int ji = 0; ji < R_NUMJOINTS; ji++) this->jangles.push_back(0);
  //for (int ji=0; ji<R_NUMJOINTS; ji++) this->jangles[ji] = 0;

  this->joint[R_RHIP].cmdZero = RHIPZERO;
  this->joint[R_RHIP].cmdMin = RHIPMIN;
  this->joint[R_RHIP].cmdMax = RHIPMAX;
  this->joint[R_RHIP].cmdDir = RHIPDIR;
  this->joint[R_RHIP].cmdMult = RHIPMULT;
  this->joint[R_RHIP].servoChannel = CH_RHIP;

  this->joint[R_LHIP].cmdZero = LHIPZERO;
  this->joint[R_LHIP].cmdMin = LHIPMIN;
  this->joint[R_LHIP].cmdMax = LHIPMAX;
  this->joint[R_LHIP].cmdDir = LHIPDIR;
  this->joint[R_LHIP].cmdMult = LHIPMULT;
  this->joint[R_LHIP].servoChannel = CH_LHIP;

  this->joint[R_RKNEE].cmdZero = RKNEEZERO;
  this->joint[R_RKNEE].cmdMin = RKNEEMIN;
  this->joint[R_RKNEE].cmdMax = RKNEEMAX;
  this->joint[R_RKNEE].cmdDir = RKNEEDIR;
  this->joint[R_RKNEE].cmdMult = RKNEEMULT;
  this->joint[R_RKNEE].servoChannel = CH_RKNEE;

  this->joint[R_LKNEE].cmdZero = LKNEEZERO;
  this->joint[R_LKNEE].cmdMin = LKNEEMIN;
  this->joint[R_LKNEE].cmdMax = LKNEEMAX;
  this->joint[R_LKNEE].cmdDir = LKNEEDIR;
  this->joint[R_LKNEE].cmdMult = LKNEEMULT;
  this->joint[R_LKNEE].servoChannel = CH_LKNEE;

  this->joint[R_RTWIST].cmdZero = RTWISTZERO;
  this->joint[R_RTWIST].cmdMin = RTWISTMIN;
  this->joint[R_RTWIST].cmdMax = RTWISTMAX;
  this->joint[R_RTWIST].cmdDir = RTWISTDIR;
  this->joint[R_RTWIST].cmdMult = RTWISTMULT;
  this->joint[R_RTWIST].servoChannel = CH_RTWIST;

  this->joint[R_LTWIST].cmdZero = LTWISTZERO;
  this->joint[R_LTWIST].cmdMin = LTWISTMIN;
  this->joint[R_LTWIST].cmdMax = LTWISTMAX;
  this->joint[R_LTWIST].cmdDir = LTWISTDIR;
  this->joint[R_LTWIST].cmdMult = LTWISTMULT;
  this->joint[R_LTWIST].servoChannel = CH_LTWIST;

  this->joint[R_EYES].cmdZero = EYESZERO;
  this->joint[R_EYES].cmdMin = EYESMIN;
  this->joint[R_EYES].cmdMax = EYESMAX;
  this->joint[R_EYES].cmdDir = EYESDIR;
  this->joint[R_EYES].cmdMult = EYESMULT;
  this->joint[R_EYES].servoChannel = CH_EYES;

  this->joint[R_RARM].cmdZero = RARMZERO;
  this->joint[R_RARM].cmdMin = RARMMIN;
  this->joint[R_RARM].cmdMax = RARMMAX;
  this->joint[R_RARM].cmdDir = RARMDIR;
  this->joint[R_RARM].cmdMult = RARMMULT;
  this->joint[R_RARM].servoChannel = CH_RARM;

  this->joint[R_LARM].cmdZero = LARMZERO;
  this->joint[R_LARM].cmdMin = LARMMIN;
  this->joint[R_LARM].cmdMax = LARMMAX;
  this->joint[R_LARM].cmdDir = LARMDIR;
  this->joint[R_LARM].cmdMult = LARMMULT;
  this->joint[R_LARM].servoChannel = CH_LARM;

  this->joint[R_AUX1].cmdZero = AUX1ZERO;
  this->joint[R_AUX1].cmdMin = AUX1MIN;
  this->joint[R_AUX1].cmdMax = AUX1MAX;
  this->joint[R_AUX1].cmdDir = AUX1DIR;
  this->joint[R_AUX1].cmdMult = AUX1MULT;
  this->joint[R_AUX1].servoChannel = CH_AUX1;

  this->joint[R_AUX2].cmdZero = AUX2ZERO;
  this->joint[R_AUX2].cmdMin = AUX2MIN;
  this->joint[R_AUX2].cmdMax = AUX2MAX;
  this->joint[R_AUX2].cmdDir = AUX2DIR;
  this->joint[R_AUX2].cmdMult = AUX2MULT;
  this->joint[R_AUX2].servoChannel = CH_AUX2;

  this->i2cptr = new myI2C();
  initServoBoard(this->i2cptr, this->deviceAddr);

  return true;
}

bool martyrobot::setServo(int ji, float angle) {
  if (ji < 0 || ji >= R_NUMJOINTS)
    return false;
  uint16_t cmd = this->jointPosToServoCmd(angle, this->joint[ji].cmdZero,
                                          this->joint[ji].cmdMult,
                                          this->joint[ji].cmdDir,
                                          this->joint[ji].cmdMax,
                                          this->joint[ji].cmdMin);
  if (DEBUG_MODE) {
    printf("setting servo on channel: %d to %d\n",
           this->joint[ji].servoChannel, cmd);
  }
  this->setServoPulse(this->joint[ji].servoChannel, cmd);
  this->jangles[ji] = angle;

  return true;
}

bool martyrobot::setServos(std::deque <float> angles) {
  int ji = 0;
  for (std::deque<float>::iterator ai = angles.begin(); ai != angles.end();
       ai++) {
    uint16_t cmd = this->jointPosToServoCmd(*ai, this->joint[ji].cmdZero,
                                            this->joint[ji].cmdMult, this->joint[ji].cmdDir, this->joint[ji].cmdMax,
                                            this->joint[ji].cmdMin);
    this->setServoPulse(this->joint[ji].servoChannel, cmd);
    this->jangles[ji] = *ai;
    ji++;
  }
  return true;
}

void martyrobot::printAngles() {
  if (DEBUG_MODE) {
    printf("Angles from inside class: \t");
    for (std::deque<float>::iterator di = this->jangles.begin();
         di != this->jangles.end(); di++) {
      printf("%2.2f\t", *di);
    }
    printf("\n");
  }
  return;
}
