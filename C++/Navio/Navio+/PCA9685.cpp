#include "PCA9685.h"

PCA9685::PCA9685(uint8_t address)
{
  this->devAddr = address;
}

void PCA9685::initialize()
{
  this->frequency = getFrequency();
  I2Cdev::writeBit(devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_AI_BIT, 1);
  restart();
}

bool PCA9685::testConnection()
{
  uint8_t data;
  int8_t status = I2Cdev::readByte(devAddr, PCA9685_RA_PRE_SCALE, &data);
  if (status > 0)
    return true;
  else
    return false;
}

void PCA9685::sleep()
{
  I2Cdev::writeBit(devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
}

void PCA9685::restart()
{
  I2Cdev::writeByte(devAddr, PCA9685_RA_MODE1, (1 << PCA9685_MODE1_SLEEP_BIT));
  I2Cdev::writeByte(
    devAddr, PCA9685_RA_MODE1, ((1 << PCA9685_MODE1_SLEEP_BIT) | (1 << PCA9685_MODE1_EXTCLK_BIT)));
  I2Cdev::writeByte(
    devAddr, PCA9685_RA_MODE1,
    ((1 << PCA9685_MODE1_RESTART_BIT) | (1 << PCA9685_MODE1_EXTCLK_BIT)
     | (1 << PCA9685_MODE1_AI_BIT)));
}

float PCA9685::getFrequency()
{
  uint8_t data;
  I2Cdev::readByte(devAddr, PCA9685_RA_PRE_SCALE, &data);
  return 24576000.f / 4096.f / (data + 1);
}

void PCA9685::setFrequency(float frequency)
{
  sleep();
  usleep(10000);
  uint8_t prescale = roundf(24576000.f / 4096.f / frequency) - 1;
  I2Cdev::writeByte(devAddr, PCA9685_RA_PRE_SCALE, prescale);
  this->frequency = getFrequency();
  restart();
}

void PCA9685::setPWM(uint8_t channel, uint16_t offset, uint16_t length)
{
  uint8_t data[4] = { 0, 0, 0, 0 };
  if (length == 0)
  {
    data[3] = 0x10;
  }
  else if (length >= 4096)
  {
    data[1] = 0x10;
  }
  else
  {
    data[0] = offset & 0xFF;
    data[1] = offset >> 8;
    data[2] = length & 0xFF;
    data[3] = length >> 8;
  }
  I2Cdev::writeBytes(devAddr, PCA9685_RA_LED0_ON_L + 4 * channel, 4, data);
}

void PCA9685::setPWM(uint8_t channel, uint16_t length)
{
  setPWM(channel, 0, length);
}

void PCA9685::setPWMmS(uint8_t channel, float length_mS)
{
  setPWM(channel, round((length_mS * 4096.f) / (1000.f / frequency)));
}

void PCA9685::setPWMuS(uint8_t channel, float length_uS)
{
  setPWM(channel, round((length_uS * 4096.f) / (1000000.f / frequency)));
}

void PCA9685::setAllPWM(uint16_t offset, uint16_t length)
{
  uint8_t data[4] = { static_cast<uint8_t>(offset & 0xFF), static_cast<uint8_t>(offset >> 8),
                      static_cast<uint8_t>(length & 0xFF), static_cast<uint8_t>(length >> 8) };
  I2Cdev::writeBytes(devAddr, PCA9685_RA_ALL_LED_ON_L, 4, data);
}

void PCA9685::setAllPWM(uint16_t length)
{
  setAllPWM(0, length);
}

void PCA9685::setAllPWMmS(float length_mS)
{
  setAllPWM(round((length_mS * 4096.f) / (1000.f / frequency)));
}

void PCA9685::setAllPWMuS(float length_uS)
{
  setAllPWM(round((length_uS * 4096.f) / (1000000.f / frequency)));
}
