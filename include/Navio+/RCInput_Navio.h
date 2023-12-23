#pragma once

#include "../Common/gpio.h"
#include "../Common/RCInput.h"

class RCInput_Navio : public RCInput
{
public:
  explicit RCInput_Navio();
  void initialize() override;
  int read(int ch) override;

private:
  void ppmOnEdge(int, int level, uint32_t tick);
  static void ppmOnEdgeTrampolin(int gpio, int level, uint32_t tick, void* userdata);

  static const uint8_t outputEnablePin = RPI_GPIO_27;

  //================================ Options =====================================

  uint32_t samplingRate = 1;         // 1 microsecond (can be 1,2,4,5,10)
  uint32_t ppmInputGpio = 4;         // PPM input on Navio's 2.54 header
  uint32_t ppmSyncLength = 4000;     // Length of PPM sync pause
  uint32_t ppmChannelsNumber = 8;    // Number of channels packed in PPM
  uint32_t servoFrequency = 50;      // Servo control frequency
  bool verboseOutputEnabled = true;  // Output channels values to console

  //================================== Data ======================================

  float channels[8];

  //============================== PPM decoder ===================================

  uint32_t currentChannel = 0;
  uint32_t previousTick;
  uint32_t deltaTime;
};
