#pragma once

#include "../Common/RCOutput.h"
#include "./PCA9685.h"

class RCOutput_Navio : public RCOutput
{
public:
  explicit RCOutput_Navio();
  bool initialize(const size_t&) override;
  bool enable(const size_t&) override;
  bool setFrequency(const size_t&, const size_t& frequency) override;
  bool setDutyCycle(const size_t& channel, const double& period) override;

private:
  PCA9685 pwm_;
};
