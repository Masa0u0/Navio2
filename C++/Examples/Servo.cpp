/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Control servos connected to PWM driver onboard of Navio2 shield for Raspberry Pi.

Connect servo to Navio2's rc output and watch it work.
PWM_OUTPUT = 0 complies to channel number 1, 1 to channel number 2 and so on.
To use full range of your servo correct SERVO_MIN and SERVO_MAX according to it's specification.

To run this example navigate to the directory containing it and run following commands:
make
sudo ./Servo
*/

#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"
#include <unistd.h>
#include <memory>

#define SERVO_MIN 1250 /*uS*/
#define SERVO_MAX 1750 /*uS*/

#define PWM_OUTPUT 0

std::unique_ptr<RCOutput> get_rcout()
{
  if (get_navio_version() == NAVIO2)
  {
    auto ptr = std::unique_ptr<RCOutput>{ new RCOutput_Navio2() };
    return ptr;
  }
  else
  {
    auto ptr = std::unique_ptr<RCOutput>{ new RCOutput_Navio() };
    return ptr;
  }
}

int main(int, char* argv[])
{
  auto pwm = get_rcout();

  if (check_apm())
  {
    return 1;
  }

  if (getuid())
  {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }

  if (!(pwm->initialize(PWM_OUTPUT)))
  {
    return 1;
  }

  pwm->setFrequency(PWM_OUTPUT, 50);

  if (!(pwm->enable(PWM_OUTPUT)))
  {
    return 1;
  }

  while (true)
  {
    if (!pwm->setDutyCycle(PWM_OUTPUT, SERVO_MIN))
    {
      fprintf(stderr, "Failed to set PWM duty cycle.\n");
    }
    sleep(1);
    if (!pwm->setDutyCycle(PWM_OUTPUT, SERVO_MAX))
    {
      fprintf(stderr, "Failed to set PWM duty cycle.\n");
    }
    sleep(1);
  }

  return 0;
}
