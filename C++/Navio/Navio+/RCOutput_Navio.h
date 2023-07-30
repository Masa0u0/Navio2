#pragma once

#include <Common/RCOutput.h>
#include "PCA9685.h"
#include <Common/gpio.h>

using namespace Navio;

class RCOutput_Navio : public RCOutput
{
public:
  RCOutput_Navio();
  bool initialize(int channel) override;
  bool enable(int channel) override;
  bool set_frequency(int channel, float frequency) override;
  bool set_duty_cycle(int channel, float period) override;

private:
  PCA9685 pwm;
};
