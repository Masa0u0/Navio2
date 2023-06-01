#include <unistd.h>
#include <memory>
#include <iostream>

#include <Navio2/PWM.h>
#include <Navio2/RCOutput_Navio2.h>
#include <Common/Util.h>

#define CHANNEL 0
#define FREQUENCY 50     // [Hz]
#define SERVO_MIN 1000   // [us]
#define SERVO_MAX 2000   // [us]
#define SERVO_LOW 1030   // [us]
#define SERVO_HIGH 1080  // [us]

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

  // Setup ESC
  // https://shizenkarasuzon.hatenablog.com/entry/2019/03/04/002326#%E8%A3%9C%E8%B6%B3ESC%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6
  if (!pwm.set_duty_cycle(CHANNEL, SERVO_MAX))
  {
    return 1;
  }
  cout << "The maximum value of the pulse is output. Connect the ESC to the battery." << endl;
  sleep(1);
  if (!pwm.set_duty_cycle(CHANNEL, SERVO_MIN))
  {
    return 1;
  }
  cout << "The minimum value of the pulse is output." << endl;

  sleep(3);
  cout << "Setup finished. Start to control motor." << endl;

  // Servo control loop
  while (true)
  {
    if (!pwm.set_duty_cycle(CHANNEL, SERVO_LOW))
    {
      cerr << "Failed to set PWM duty cycle." << endl;
    }
    sleep(1);
    if (!pwm.set_duty_cycle(CHANNEL, SERVO_HIGH))
    {
      cerr << "Failed to set PWM duty cycle." << endl;
    }
    sleep(1);
  }

  return 0;
}
