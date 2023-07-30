#pragma once

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>

class PWM
{
public:
  explicit PWM();

  bool init(unsigned int channel);
  bool enable(unsigned int channel);
  bool set_period(unsigned int channel, unsigned int freq);
  /* Note: period [ms] */
  bool set_duty_cycle(unsigned int channel, float period);
};
