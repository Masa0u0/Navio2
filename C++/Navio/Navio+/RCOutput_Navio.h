#pragma once

#include "../Common/RCOutput.h"
#include "./PCA9685.h"

class RCOutput_Navio : public RCOutput
{
public:
  explicit RCOutput_Navio();
  bool initialize(int) override;
  bool enable(int) override;
  bool set_frequency(int, float frequency) override;
  bool set_duty_cycle(int channel, float period) override;

private:
  PCA9685 pwm;
};
