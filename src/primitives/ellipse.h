#ifndef PRIMITIVES__ELLIPSE_H_
#define PRIMITIVES__ELLIPSE_H_

#include "../types.h"

class Ellipse {
 public:
  Ellipse(float a, float b, float c, float d, float e, float f,
          float fitting_error);

 public:
  float fitting_error() { return fitting_error_; }

 public:
  static Ellipse FitFromEdgeSegment(const EdgeSegment &edge_segment);

 protected:
  float ComputeError(Position pos);

 public:
  float parameters_[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float cx_ = 0.0f;
  float cy_ = 0.0f;
  float axis_lengths_[2] = {0.0f, 0.0f};
  float angle_ = 0.0f;

  float fitting_error_ = 0.0f;
};

#endif
