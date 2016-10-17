#include "servoboardutils.h"

int setPWM(myI2C* i2cptr, unsigned char deviceAddr, uint8_t channel,
           uint16_t pos) {
  i2cptr->Send_I2C_2Bytes(deviceAddr, LED0_OFF_L + (4 * channel), pos & 0xFF,
                          (pos >> 8) & 0xFF);
  return 1;
}

int setAllPWM(myI2C* i2cptr, unsigned char deviceAddr, uint16_t on,
              uint16_t off) {
  //  "Sets a all PWM channels"
  i2cptr->Send_I2C_Byte(deviceAddr, ALL_LED_ON_L, on & 0xff);
  i2cptr->Send_I2C_Byte(deviceAddr, ALL_LED_ON_H, (on >> 8) & 0xff);
  i2cptr->Send_I2C_Byte(deviceAddr, ALL_LED_OFF_L, off & 0xff);
  i2cptr->Send_I2C_Byte(deviceAddr, ALL_LED_OFF_H, (off >> 8) & 0xff);
  return 1;
}

int initServoBoard(myI2C* i2cptr, unsigned char deviceAddr) {
  // Set all pwm off
  setAllPWM(i2cptr, deviceAddr, 0, 0);

  // It is necessary to put the servo controller into sleep mode before setting the prescaler (i.e. output frequency)
  uint8_t mode1 = i2cptr->Read_I2C_Byte(deviceAddr, MODE1); // Read current mode
  mode1 |= SLEEP;             // set sleep bit
  i2cptr->Send_I2C_Byte(deviceAddr, MODE1, mode1);    // Set sleep bit on device
  usleep(5000);               // Pause for a bit
  // Set output frequency to 50Hz. Note actual value of 127 tweaked based on scope readings
  i2cptr->Send_I2C_Byte(deviceAddr, PRESCALE, 127);

  i2cptr->Send_I2C_Byte(deviceAddr, MODE2, OUTDRV);   // initialisation
  i2cptr->Send_I2C_Byte(deviceAddr, MODE1, ALLCALL | AI);
  usleep(5000);

  // and take the controller out of sleep mode
  mode1 = i2cptr->Read_I2C_Byte(deviceAddr, MODE1);
  mode1 &= ~SLEEP;
  i2cptr->Send_I2C_Byte(deviceAddr, MODE1, mode1);
  usleep(5000);

  return 1;   // TODO: PROPER ERROR DETECTION HERE
}
