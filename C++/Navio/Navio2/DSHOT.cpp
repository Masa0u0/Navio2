#include <stdexcept>
#include <cassert>
#include <pigpio/pigpio.h>

#include "./DSHOT.h"

using namespace std;

DSHOT::DSHOT(uint32_t dshot_type)
{
  switch (dshot_type)
  {
    case DSHOT_150:
      sleep_high_ = 5.0e-6;
      sleep_low_ = 2.5e-6;
      break;
    case DSHOT_300:
      sleep_high_ = 2.5e-6;
      sleep_low_ = 1.25e-6;
      break;
    case DSHOT_600:
      sleep_high_ = 1.25e-6;
      sleep_low_ = 6.25e-7;
      break;
    case DSHOT_1200:
      sleep_high_ = 6.25e-7;
      sleep_low_ = 3.125e-7;
      break;
    default:
      throw runtime_error("Unknown DSHOT type: " + to_string(dshot_type));
  }

  if (gpioInitialise() < 0)
  {
    throw runtime_error("Failed to initialize GPIO.");
  }
}

DSHOT::~DSHOT()
{
  gpioTerminate();
}

void DSHOT::init(uint32_t pin)
{
  gpioSetMode(pin, PI_OUTPUT);
}

void DSHOT::setSignal(uint32_t pin, uint32_t throttle, uint32_t telem)
{
  assert(throttle < (1 << 11));

  uint32_t thr_telem = addTelemetry(throttle, telem);
  uint32_t thr_telem_ck = addChecksum(thr_telem);
  sendPacket(pin, thr_telem_ck);
}

uint32_t DSHOT::addTelemetry(uint32_t throttle, uint32_t telem)
{
  return (throttle << 1) | (telem & 1);
}

uint32_t DSHOT::addChecksum(uint32_t thr_telem)
{
  uint32_t ck = 0;
  uint32_t ck_data = thr_telem;
  for (uint32_t i = 0; i < 3; ++i)
  {
    ck ^= ck_data;
    ck_data >>= 4;
  }
  ck &= 0xf;
  uint32_t packet_ck = (thr_telem << 4) | ck;
  return packet_ck;
}

void DSHOT::sendPacket(uint32_t pin, uint32_t packet)
{
  for (uint32_t i = 15; i >= 0; --i)
  {
    if ((packet >> i) & 1)
    {
      sendHigh(pin);
    }
    else
    {
      sendLow(pin);
    }
  }
}

void DSHOT::sendHigh(uint32_t pin)
{
  gpioWrite(pin, 1);
  time_sleep(sleep_high_);
}

void DSHOT::sendLow(uint32_t pin)
{
  gpioWrite(pin, 0);
  time_sleep(sleep_low_);
}
