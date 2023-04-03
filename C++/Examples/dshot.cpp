#include <unistd.h>
#include <iostream>

#include <Common/Util.h>
#include <Navio2/DSHOT.h>

#define PERIOD 10  // [s]

using namespace std;

int main(int argc, char* argv[])
{
  if (check_apm())
  {
    cerr << "check_apm() failed." << endl;
    return 1;
  }

  if (getuid())
  {
    cerr << "Not root." << endl;
    return 1;
  }

  if (argc != 2)
  {
    cerr << "Please specify pin number like: " << argv[0] << " 1" << endl;
    return 1;
  }

  DSHOT dshot(DSHOT::DSHOT_600);

  uint32_t pin = *argv[1] - '0';  // char -> int
  dshot.initialize(pin);

  constexpr uint32_t cycle_count = ((1 << 11) - 48) * 2;
  constexpr uint32_t sleep_time = PERIOD * 1000000 / cycle_count;  // [us]

  while (true)
  {
    for (int throttle = 48; throttle < (1 << 11); ++throttle)
    {
      dshot.setSignal(pin, throttle);
      usleep(sleep_time);
    }
    for (int throttle = (1 << 11) - 1; throttle >= 48; --throttle)
    {
      dshot.setSignal(pin, throttle);
      usleep(sleep_time);
    }
  }
}
