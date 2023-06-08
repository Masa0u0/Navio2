#include <unistd.h>
#include <memory>
#include <iostream>

#include <Navio2/PWM.h>
#include <Navio2/RCOutput_Navio2.h>
#include <Common/Util.h>

#define CHANNEL 0
#define FREQUENCY 50       // [Hz]
#define PWM_MAX 2000       // [us]
#define PWM_MIN 1000       // [us]
#define SLEEP_TIME_HIGH 3  // [s]
#define SLEEP_TIME_LOW 4   // [s]

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
  sleep(SLEEP_TIME_HIGH);

  cout << "Step 2: Send minimum throttle command" << endl;
  if (!pwm.set_duty_cycle(CHANNEL, PWM_MIN))
  {
    return 1;
  }
  sleep(SLEEP_TIME_LOW);

  cout << "Now the ESC should be calibrated." << endl;
  return 0;
}
