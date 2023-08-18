#pragma once

#include <cinttypes>

class PWM
{
public:
  explicit PWM();

  bool init(uint32_t channel);
  bool enable(uint32_t channel);
  bool set_period(uint32_t channel, uint32_t freq);
  /* Note: period [ms] */
  bool set_duty_cycle(uint32_t channel, float period);
};
