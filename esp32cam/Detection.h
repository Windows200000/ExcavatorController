#pragma once

struct Detection {
  String label;
  float  x, y, w, h;
  float  confidence;
};