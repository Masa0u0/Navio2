#pragma once

#include <unordered_set>
/**
 * @brief DSHOT for Raspberry Pi
 * cf. https://github.com/dmrlawson/raspberrypi-dshot
 * cf. https://elinux.org/RPi_GPIO_Code_Samples#pigpio
 */
class DSHOT
{
public:
  enum
  {
    DSHOT_150,
    DSHOT_300,
    DSHOT_600,
  };

  DSHOT(uint32_t dshot_type);
  ~DSHOT();

  void initialize(uint32_t pin);
  void setSignal(uint32_t pin, uint32_t throttle, uint32_t telem = 0);

private:
  timespec t0_high_;
  timespec t0_low_;
  timespec t1_high_;
  timespec t1_low_;
  std::unordered_set<uint32_t> gpio_pins_;

  uint32_t getBCM(uint32_t pin);
  void exportGPIO(uint32_t bcm);
  void unexportGPIO(uint32_t bcm);
  void setGpioOutput(uint32_t bcm);
  uint32_t addTelemetry(uint32_t throttle, uint32_t telem);
  uint32_t addChecksum(uint32_t thr_telem);
  void sendPacket(uint32_t bcm, uint32_t packet);
  void sendOne(uint32_t bcm);
  void sendZero(uint32_t bcm);
  void setHigh(uint32_t bcm);
  void setLow(uint32_t bcm);
};
