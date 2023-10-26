#pragma once

#include "../Common/RCOutput.h"
#include "./PWM.h"

class RCOutput_Navio2 : public RCOutput
{
public:
  inline explicit RCOutput_Navio2();

  inline bool initialize(const uint32_t& channel) override;
  inline bool enable(const uint32_t& channel) override;
  inline bool setFrequency(const uint32_t& channel, const uint32_t& frequency) override;
  inline bool setDutyCycle(const uint32_t& channel, const double& period_ns) override;

private:
  PWM pwm_;
};

inline RCOutput_Navio2::RCOutput_Navio2()
{
}

inline bool RCOutput_Navio2::initialize(const uint32_t& channel)
{
  return pwm_.init(channel);
}

inline bool RCOutput_Navio2::enable(const uint32_t& channel)
{
  return pwm_.enable(channel);
}

inline bool RCOutput_Navio2::setFrequency(const uint32_t& channel, const uint32_t& frequency)
{
  return pwm_.setPeriod(channel, frequency);
}

inline bool RCOutput_Navio2::setDutyCycle(const uint32_t& channel, const double& period_ns)
{
  return pwm_.setDutyCycle(channel, period_ns / 1000);
}
