#include "../../include/Common/I2Cdev.h"
#include "../../include/Navio+/MB85RC256.h"

MB85RC256::MB85RC256(uint8_t address)
{
  this->device_address = address;
}

uint8_t MB85RC256::readByte(uint16_t register_address, uint8_t* data)
{
  return MB85RC256::readBytes(register_address, 1, data);
}

uint8_t MB85RC256::writeByte(uint16_t register_address, uint8_t data)
{
  return MB85RC256::writeBytes(register_address, 1, &data);
}

uint8_t MB85RC256::writeBytes(uint16_t register_address, uint8_t length, uint8_t* data)
{
  uint8_t reg_address_low = register_address;        // lower part of the address
  uint8_t reg_address_high = register_address >> 8;  // higher part of the address
  uint8_t msg[length + 1];

  msg[0] = reg_address_low;

  for (int i = 0; i < length; ++i)
  {
    msg[i + 1] = data[i];
  }

  return I2Cdev::writeBytes(this->device_address, reg_address_high, length + 1, msg);
}

uint8_t MB85RC256::readBytes(uint16_t register_address, uint8_t length, uint8_t* data)
{
  uint8_t reg_address_low = register_address;        // lower part of the address
  uint8_t reg_address_high = register_address >> 8;  // higher part of the address

  // set the read pointer to the desired address
  I2Cdev::writeByte(this->device_address, reg_address_high, reg_address_low);

  return I2Cdev::readBytesNoRegAddress(this->device_address, length, data);
}
