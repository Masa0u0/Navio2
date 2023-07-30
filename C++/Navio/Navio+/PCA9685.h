#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <string>

#include "../Common/I2Cdev.h"

#define PCA9685_DEFAULT_ADDRESS 0x40  // All address pins low, Navio default

#define PCA9685_RA_MODE1 0x00
#define PCA9685_RA_MODE2 0x01
#define PCA9685_RA_LED0_ON_L 0x06
#define PCA9685_RA_LED0_ON_H 0x07
#define PCA9685_RA_LED0_OFF_L 0x08
#define PCA9685_RA_LED0_OFF_H 0x09
#define PCA9685_RA_ALL_LED_ON_L 0xFA
#define PCA9685_RA_ALL_LED_ON_H 0xFB
#define PCA9685_RA_ALL_LED_OFF_L 0xFC
#define PCA9685_RA_ALL_LED_OFF_H 0xFD
#define PCA9685_RA_PRE_SCALE 0xFE

#define PCA9685_MODE1_RESTART_BIT 7
#define PCA9685_MODE1_EXTCLK_BIT 6
#define PCA9685_MODE1_AI_BIT 5
#define PCA9685_MODE1_SLEEP_BIT 4
#define PCA9685_MODE1_SUB1_BIT 3
#define PCA9685_MODE1_SUB2_BIT 2
#define PCA9685_MODE1_SUB3_BIT 1
#define PCA9685_MODE1_ALLCALL_BIT 0

#define PCA9685_MODE2_INVRT_BIT 4
#define PCA9685_MODE2_OCH_BIT 3
#define PCA9685_MODE2_OUTDRV_BIT 2
#define PCA9685_MODE2_OUTNE1_BIT 1
#define PCA9685_MODE2_OUTNE0_BIT 0

class PCA9685
{
public:
  PCA9685(uint8_t address = PCA9685_DEFAULT_ADDRESS);

  void initialize();
  bool testConnection();

  float getFrequency();
  void setFrequency(float frequency);

  void sleep();
  void restart();

  void setPWM(uint8_t channel, uint16_t offset, uint16_t length);
  void setPWM(uint8_t channel, uint16_t length);
  void setPWMmS(uint8_t channel, float length_mS);
  void setPWMuS(uint8_t channel, float length_uS);

  void setAllPWM(uint16_t offset, uint16_t length);
  void setAllPWM(uint16_t length);
  void setAllPWMmS(float length_mS);
  void setAllPWMuS(float length_uS);

private:
  uint8_t devAddr;
  float frequency;
};
