#ifndef __ARCHIE_CALIBRATION
#define __ARCHIE_CALIBRATION

#include "martyJointCalib.h"

// I2C address of servo board
#define I2CADDRESS  0x40

// motors - PWM channels servos are attached to
enum PWMChannels {
  CH_LHIP =   0,
  CH_LTWIST = 1,
  CH_LKNEE =  2,
  CH_RHIP =   3,
  CH_RTWIST = 4,
  CH_RKNEE =  5,
  CH_LARM =   6,
  CH_RARM =   7,
  CH_EYES =   8,
  CH_AUX1 =   9,
  CH_AUX2 =   10,
  CH_AUX3 =   11,
  CH_NUM
};

// #define CH_EYER   6
// #define EYERZERO  270
// #define EYERMAX   310
// #define EYERMIN   240
// #define EYERDIR   -1
// #define EYERMULT  1

# define HIPOFFSET 100
# define TWISTOFFSET 180
# define KNEEOFFSET 110
# define ARMOFFSET 190
# define EYESOFFSET 100
# define AUXOFFSET 190

// Servo polarity and multiplier
#define RHIPDIR   -1
#define RHIPMULT  1

#define RTWISTDIR   1
#define RTWISTMULT  1

#define RKNEEDIR  1
#define RKNEEMULT 1

#define LHIPDIR   1
#define LHIPMULT  1

#define LTWISTDIR   1
#define LTWISTMULT  1

#define LKNEEDIR  1
#define LKNEEMULT 1

#define LARMDIR   1
#define LARMMULT    1

#define RARMDIR   -1
#define RARMMULT    1

#define AUX1DIR   1
#define AUX1MULT  1

#define EYESDIR   1
#define EYESMULT  1

#define AUX2DIR   1
#define AUX2MULT  1

// These are "angles" rather than servo commands
#define EYESNORMAL  0
#define EYESANGRY   22
#define EYESEXCITED -20
#define EYESWIDE  -50


#endif
