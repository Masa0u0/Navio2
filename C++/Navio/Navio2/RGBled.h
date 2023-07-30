#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdint>

#include "Common/gpio.h"
#include "Common/Led.h"

class RGBled
{
public:
  RGBled();

  bool initialize();
  void setColor(Colors color);

private:
  Navio::Pin* pinR;
  Navio::Pin* pinG;
  Navio::Pin* pinB;
};
