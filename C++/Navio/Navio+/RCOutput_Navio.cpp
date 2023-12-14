#include "../Common/gpio.h"
#include "./RCOutput_Navio.h"

using namespace Navio;

RCOutput_Navio::RCOutput_Navio()
{
}

bool RCOutput_Navio::initialize(const size_t&)
{
  static constexpr uint8_t outputEnablePin = RPI_GPIO_27;

  Pin pin(outputEnablePin);

  if (pin.init())
  {
    pin.setMode(Pin::GpioModeOutput);
    pin.write(0);  // drive Output Enable low
  }
  else
  {
    return false;
  }

  return true;
}

bool RCOutput_Navio::enable(const size_t&)
{
  pwm_.initialize();
  return true;
}

bool RCOutput_Navio::setFrequency(const size_t&, const size_t& frequency)
{
  pwm_.setFrequency(frequency);
  return true;
}

bool RCOutput_Navio::setDutyCycle(const size_t& channel, const double& period)
{
  pwm_.setPWMmS(channel + 3, period / 1000);  // 1st Navio RC output is 3
  return true;
}
