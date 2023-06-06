#include <unistd.h>
#include <memory>
#include <iostream>

#include <Navio2/PWM.h>
#include <Navio2/RCOutput_Navio2.h>
#include <Common/Util.h>

#define CHANNEL 0
#define FREQUENCY 50  // [Hz]
#define PWM_MIN 1000  // [us]
#define PWM_MAX 2000  // [us]

using namespace std;

int main(int argc, char* argv[])
{
  // Check the execution environment
  if (check_apm())
  {
    return 1;
  }
  if (getuid())
  {
    cerr << "Not root. Please launch like this: sudo " << argv[0] << endl;
    return 1;
  }

  // Initialize PWM handler
  RCOutput_Navio2 pwm;
  if (!(pwm.initialize(CHANNEL)))
  {
    return 1;
  }
  if (!pwm.set_frequency(CHANNEL, FREQUENCY))
  {
    return 1;
  }
  if (!(pwm.enable(CHANNEL)))
  {
    return 1;
  }

  // Calibrate
  cout << "Step 1: Send maximum throttle command" << endl;
  if (!pwm.set_duty_cycle(CHANNEL, PWM_MAX))
  {
    return 1;
  }
  sleep(2);

  cout << "Step 2: Send minimum throttle command" << endl;
  if (!pwm.set_duty_cycle(CHANNEL, PWM_MIN))
  {
    return 1;
  }
  sleep(1);

  cout << "Step 3: Confirmation" << endl;
  if (!pwm.set_duty_cycle(CHANNEL, PWM_MIN))
  {
    return 1;
  }
  sleep(1);

  // Now the ESC should be calibrated.

  return 0;
}
