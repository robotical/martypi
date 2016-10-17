#ifndef __SERVOBOARDUTILS__
#define __SERVOBOARDUTILS__

#include <stdint.h>
#include "i2c.h"

// I2C BOARD DEFINITIONS
// Define register address
#define MODE1           0x00
#define MODE2           0x01
#define SUBADR1         0x02
#define SUBADR2         0x03
#define SUBADR3         0x04
#define PRESCALE        0xFE
#define LED0_ON_L       0x06
#define LED0_ON_H       0x07
#define LED0_OFF_L      0x08
#define LED0_OFF_H      0x09
#define ALL_LED_ON_L    0xFA
#define ALL_LED_ON_H    0xFB
#define ALL_LED_OFF_L   0xFC
#define ALL_LED_OFF_H   0xFD
//  # Bits
#define RESTART         0x80
#define SLEEP           0x10
#define ALLCALL         0x01
#define INVRT           0x10
#define OUTDRV          0x04
#define AI        0x20

int setPWM(myI2C* i2cptr, unsigned char deviceAddr, uint8_t channel,
           uint16_t pos);
int setAllPWM(myI2C* i2cptr, unsigned char deviceAddr, uint16_t on,
              uint16_t off);
int initServoBoard(myI2C* i2cptr, unsigned char deviceAddr);

#endif
