#pragma once

#include "../Common/RCOutput.h"
#include "./PWM.h"

class RCOutput_Navio2 : public RCOutput
{
public:
  explicit RCOutput_Navio2();
  bool initialize(const uint32_t& channel) override;
  bool enable(const uint32_t& channel) override;
  bool setFrequency(const uint32_t& channel, const float& frequency) override;
  /* Note: period [us] */
  bool setDutyCycle(const uint32_t& channel, const float& period) override;

private:
  PWM pwm;
};
