#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <Common/I2Cdev.h>

class MB85RC256
{
  uint8_t device_address;

public:
  MB85RC256(uint8_t address = 0b1010000);
  uint8_t readByte(uint16_t register_address, uint8_t* data);
  uint8_t writeByte(uint16_t register_address, uint8_t data);
  uint8_t readBytes(uint16_t register_address, uint8_t length, uint8_t* data);
  uint8_t writeBytes(uint16_t register_address, uint8_t length, uint8_t* data);
};
