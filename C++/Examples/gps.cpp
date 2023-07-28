#include <vector>
#include <iostream>

#include <Common/Ublox.h>
#include <Common/Util.h>

#define MEASUREMENT_RATE 100  // [ms]
#define SLEEP_TIME 200        // [us]

using namespace std;

int main(int argc, char* argv[])
{
  if (check_apm())
  {
    return 1;
  }

  // Ublox class instance
  Ublox gps;

  // Payloads
  NavPayload_POSLLH posllh;
  NavPayload_STATUS status;
  NavPayload_PVT pvt;
  NavPayload_VELNED velned;
  NavPayload_COV cov;

  // Configure GPS messages
  gps.enableNavMsg(Ublox::NAV_POSLLH);
  gps.enableNavMsg(Ublox::NAV_STATUS);
  gps.disableNavMsg(Ublox::NAV_PVT);
  gps.disableNavMsg(Ublox::NAV_VELNED);
  gps.disableNavMsg(Ublox::NAV_COV);

  if (gps.configureSolutionRate(MEASUREMENT_RATE) < 0)
  {
    cerr << "Setting new rate: FAILED" << endl;
    return 1;
  }

  uint32_t cnt = 0;
  while (true)
  {
    const auto msg_id = gps.update();

    switch (msg_id)
    {
      case Ublox::NAV_POSLLH:
        gps.decode(posllh);
        cout << "NAV_POSLLH:" << endl << posllh << endl;
        break;
      case Ublox::NAV_STATUS:
        gps.decode(status);
        cout << "NAV_STATUS:" << endl << status << endl;
        break;
      case Ublox::NAV_PVT:
        gps.decode(pvt);
        cout << "NAV_PVT:" << endl << pvt << endl;
        break;
      case Ublox::NAV_VELNED:
        gps.decode(velned);
        cout << "NAV_VELNED:" << endl << velned << endl;
        break;
      case Ublox::NAV_COV:
        gps.decode(cov);
        cout << "NAV_COV:" << endl << cov << endl;
        break;
      default:
        break;
    }

    usleep(SLEEP_TIME);
  }

  return 0;
}
