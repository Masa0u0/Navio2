#pragma once

#include "../Common/ADC.h"
#include "../Common/Util.h"
#include "./ADS1115.h"

class ADC_Navio : public ADC
{
public:
  explicit ADC_Navio();
  void initialize() override;
  int get_channel_count(void) override;
  int read(int ch) override;

private:
  ADS1115 adc;
  uint16_t muxes[4] = { ADS1115_MUX_P0_NG, ADS1115_MUX_P1_NG, ADS1115_MUX_P2_NG,
                        ADS1115_MUX_P3_NG };
  float results[ARRAY_SIZE(muxes)] = { 0.0f };
};
