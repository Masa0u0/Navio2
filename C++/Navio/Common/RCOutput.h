#pragma once

#include <cinttypes>

class RCOutput
{
public:
  virtual bool initialize(const uint32_t& channel) = 0;
  virtual bool enable(const uint32_t& channel) = 0;
  virtual bool setFrequency(const uint32_t& channel, const uint32_t& frequency) = 0;
  virtual bool setDutyCycle(const uint32_t& channel, const double& period_us) = 0;
};
