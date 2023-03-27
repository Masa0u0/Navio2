#pragma once

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
    DSHOT_1200,
  };

  DSHOT(uint32_t dshot_type);
  ~DSHOT();

  void init(uint32_t pin);
  void setSignal(uint32_t pin, uint32_t throttle, uint32_t telem = 0);

private:
  double sleep_high_;
  double sleep_low_;

  uint32_t addTelemetry(uint32_t throttle, uint32_t telem);
  uint32_t addChecksum(uint32_t thr_telem);
  void sendPacket(uint32_t pin, uint32_t packet);
  void sendHigh(uint32_t pin);
  void sendLow(uint32_t pin);
};
