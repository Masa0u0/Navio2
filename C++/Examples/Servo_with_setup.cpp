#include <unistd.h>
#include <memory>
#include <iostream>

#include <Navio2/PWM.h>
#include <Navio2/RCOutput_Navio2.h>
#include <Common/Util.h>

#define CHANNEL 0
#define FREQUENCY 50    // [Hz]
#define PWM_DISARM 900  // [us]
#define PWM_LOW 1030    // [us]
#define PWM_HIGH 1080   // [us]

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

  sleep(3);
  cout << "ESC setup starts in 3 seconds." << endl;

  // Send disarm command
  cout << "Send disarm command first" << endl;
  if (!pwm.set_duty_cycle(CHANNEL, PWM_DISARM))
  {
    return 1;
  }
  sleep(3);

  cout << "Start to send PWM signal to pin " << CHANNEL + 1 << endl;

  // Servo control loop
  while (true)
  {
    if (!pwm.set_duty_cycle(CHANNEL, PWM_LOW))
    {
      cerr << "Failed to set PWM duty cycle." << endl;
    }
    sleep(1);
    if (!pwm.set_duty_cycle(CHANNEL, PWM_HIGH))
    {
      cerr << "Failed to set PWM duty cycle." << endl;
    }
    sleep(1);
  }

  return 0;
}
