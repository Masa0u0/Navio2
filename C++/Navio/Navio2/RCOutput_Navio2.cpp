#include "./RCOutput_Navio2.h"

RCOutput_Navio2::RCOutput_Navio2()
{
}

bool RCOutput_Navio2::initialize(const uint32_t& channel)
{
  return pwm.init(channel);
}

bool RCOutput_Navio2::enable(const uint32_t& channel)
{
  return pwm.enable(channel);
}

bool RCOutput_Navio2::setFrequency(const uint32_t& channel, const float& frequency)
{
  return pwm.setPeriod(channel, frequency);
}

bool RCOutput_Navio2::setDutyCycle(const uint32_t& channel, const float& period)
{
  return pwm.setDutyCycle(channel, period / 1000);
}
