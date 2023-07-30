#pragma once

#include <Common/RCOutput.h>
#include "PCA9685.h"
#include <Common/gpio.h>

using namespace Navio;

class RCOutput_Navio : public RCOutput
{
public:
  RCOutput_Navio();
  bool initialize(int) override;
  bool enable(int) override;
  bool set_frequency(int, float frequency) override;
  bool set_duty_cycle(int channel, float period) override;

private:
  PCA9685 pwm;
};
