#pragma once

#include <cinttypes>

class PWM
{
public:
  explicit PWM();

  bool init(const uint32_t& channel);
  bool enable(const uint32_t& channel);
  bool setPeriod(const uint32_t& channel, const uint32_t& freq);
  bool setDutyCycle(const uint32_t& channel, const double& period_ms);
};
