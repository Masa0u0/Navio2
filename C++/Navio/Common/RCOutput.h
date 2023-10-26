#pragma once

#include <cinttypes>

class RCOutput
{
public:
  virtual bool initialize(const uint32_t& channel) = 0;
  virtual bool enable(const uint32_t& channel) = 0;
  virtual bool setFrequency(const uint32_t& channel, const double& frequency) = 0;
  virtual bool setDutyCycle(const uint32_t& channel, const double& period) = 0;
};
