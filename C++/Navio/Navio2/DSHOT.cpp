#include <stdexcept>
#include <cassert>
#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <limits.h>

#include "./DSHOT.h"
#include "../Common/Util.h"

using namespace std;

DSHOT::DSHOT(uint32_t dshot_type)
{
  t0_high_.tv_sec = 0;
  t0_low_.tv_sec = 0;
  t1_high_.tv_sec = 0;
  t1_low_.tv_sec = 0;

  uint32_t pulse_width;

  switch (dshot_type)
  {
    case DSHOT_150:
      pulse_width = 1000000 / 150;
      t0_high_.tv_nsec = 2500;
      t1_high_.tv_nsec = 5000;
      break;
    case DSHOT_300:
      pulse_width = 1000000 / 300;
      t0_high_.tv_nsec = 1250;
      t1_high_.tv_nsec = 2500;
      break;
    case DSHOT_600:
      pulse_width = 1000000 / 600;
      t0_high_.tv_nsec = 625;
      t1_high_.tv_nsec = 1250;
      break;
    default:
      throw runtime_error("Invalid DSHOT type: " + to_string(dshot_type));
  }

  t0_low_.tv_nsec = pulse_width - t0_high_.tv_nsec;
  t1_low_.tv_nsec = pulse_width - t1_high_.tv_nsec;
}

void DSHOT::initialize(uint32_t pin)
{
  auto bcm = getBCM(pin);
  exportServoPinAsGpio(bcm);
  setGpioOutput(bcm);
}

void DSHOT::exportServoPinAsGpio(uint32_t bcm)
{
  char* file_path = "/sys/class/gpio/export";
  if (write_file(file_path, "%u", bcm) != 0)
  {
    throw runtime_error("Failed to export GPIO: " + to_string(bcm));
  }
}

void DSHOT::setGpioOutput(uint32_t bcm)
{
  string file_path = "/sys/class/gpio/gpio" + to_string(bcm) + "/direction";
  if (write_file(file_path.c_str(), "out") != 0)
  {
    throw runtime_error("Failed to set GPIO output: " + to_string(bcm));
  }
}

void DSHOT::setSignal(uint32_t pin, uint32_t throttle, uint32_t telem)
{
  assert(48 <= throttle && throttle < (1 << 11));

  uint32_t thr_telem = addTelemetry(throttle, telem);
  uint32_t thr_telem_ck = addChecksum(thr_telem);
  uint32_t bcm = getBCM(pin);
  sendPacket(bcm, thr_telem_ck);
}

char DSHOT::getBCM(uint32_t pin)
{
  return 500 + pin - 1;  // 仮想的なBCM番号
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

void DSHOT::sendPacket(uint32_t bcm, uint32_t packet)
{
  for (uint32_t i = 15; i >= 0; --i)
  {
    if ((packet >> i) & 1)
    {
      sendOne(bcm);
    }
    else
    {
      sendZero(bcm);
    }
  }
}

void DSHOT::sendOne(uint32_t bcm)
{
  setHigh(bcm);
  if (nanosleep(&t1_high_, NULL) != 0)
  {
    throw runtime_error("Error occurred during sleep.");
  }

  setLow(bcm);
  if (nanosleep(&t1_low_, NULL) != 0)
  {
    throw runtime_error("Error occurred during sleep.");
  }
}

void DSHOT::sendZero(uint32_t bcm)
{
  setHigh(bcm);
  if (nanosleep(&t0_high_, NULL) != 0)
  {
    throw runtime_error("Error occurred during sleep.");
  }

  setLow(bcm);
  if (nanosleep(&t0_low_, NULL) != 0)
  {
    throw runtime_error("Error occurred during sleep.");
  }
}

void DSHOT::setHigh(uint32_t bcm)
{
  string file_path = "/sys/class/gpio/gpio" + to_string(bcm) + "/value";
  if (write_file(file_path.c_str(), "1") != 0)
  {
    throw runtime_error("Failed to set high: " + to_string(bcm));
  }
}

void DSHOT::setLow(uint32_t bcm)
{
  string file_path = "/sys/class/gpio/gpio" + to_string(bcm) + "/value";
  if (write_file(file_path.c_str(), "0") != 0)
  {
    throw runtime_error("Failed to set low: " + to_string(bcm));
  }
}
