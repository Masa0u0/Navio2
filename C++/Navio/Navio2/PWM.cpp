#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>

#include "../Common/Util.h"
#include "./PWM.h"

PWM::PWM()
{
}

bool PWM::init(const uint32_t& channel)
{
  const int err = write_file("/sys/class/pwm/pwmchip0/export", "%u", channel);
  if (err >= 0 || err == -EBUSY)
  {
    return true;
  }
  else
  {
    printf("Can't init channel %u\n", channel);
    return false;
  }
  return true;
}

bool PWM::enable(const uint32_t& channel)
{
  char path[60] = "/sys/class/pwm/pwmchip0";
  char path_ch[20];
  sprintf(path_ch, "/pwm%u/enable", channel);
  strcat(path, path_ch);

  if (write_file(path, "1") < 0)
  {
    printf("Can't enable channel %u\n", channel);
    return false;
  }
  return true;
}

bool PWM::setPeriod(const uint32_t& channel, const uint32_t& freq)
{
  char path[60] = "/sys/class/pwm/pwmchip0";
  char path_ch[20];
  sprintf(path_ch, "/pwm%u/period", channel);
  strcat(path, path_ch);

  const int period_ns = 1e+9 / freq;
  if (write_file(path, "%u", period_ns) < 0)
  {
    printf("Can't set period to channel %u\n", channel);
    return false;
  }
  return true;
}

bool PWM::setDutyCycle(const uint32_t& channel, const float& period)
{
  char path[60] = "/sys/class/pwm/pwmchip0";
  char path_ch[20];
  sprintf(path_ch, "/pwm%u/duty_cycle", channel);
  strcat(path, path_ch);

  const int period_ns = period * 1e+6;
  if (write_file(path, "%u", period_ns) < 0)
  {
    printf("Can't set duty cycle to channel %u\n", channel);
    return false;
  }
  return true;
}
