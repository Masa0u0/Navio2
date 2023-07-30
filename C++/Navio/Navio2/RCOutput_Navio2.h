#pragma once

#include "PWM.h"
#include <Common/RCOutput.h>

class RCOutput_Navio2 : public RCOutput
{
public:
  explicit RCOutput_Navio2();
  bool initialize(int channel) override;
  bool enable(int channel) override;
  bool set_frequency(int channel, float frequency) override;
  /* Note: period [us] */
  bool set_duty_cycle(int channel, float period) override;

private:
  PWM pwm;
};
