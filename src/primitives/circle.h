#ifndef PRIMITIVES__CIRCLE_H_
#define PRIMITIVES__CIRCLE_H_

#include "../types.h"

class Circle {
 public:
  Circle(float center_x, float center_y, float radius, float fitting_error_);

 public:
  float fitting_error() { return fitting_error_; }

 public:
  static Circle FitFromEdgeSegment(const EdgeSegment &edge_segment);

 public:
  float parameters_[3] = {0.0f, 0.0f, 0.0f};
  float fitting_error_ = 0.0f;
};

#endif
