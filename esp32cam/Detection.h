#pragma once
#include <Arduino.h>

struct Detection {
  String label;
  float  x, y, w, h;   // normalised 0..1 (top-left origin)
  float  confidence;
};
