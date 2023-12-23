#pragma once

#include "../Common/Led.h"
#include "./PCA9685.h"

class Led_Navio : public Led
{
public:
  explicit Led_Navio();
  bool initialize() override;
  void setColor(Colors color) override;

private:
  PCA9685 led;
};
