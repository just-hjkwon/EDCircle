#ifndef PRIMITIVES__LINE_H_
#define PRIMITIVES__LINE_H_

#include "../types.h"

class Line {
 public:
  Line(float a, float b, float fitting_error, bool is_parameter_of_x,
       EdgeSegment edge_segment_);

 public:
  Position begin() const;
  Position end() const;
  Position vector() const;

  float get_length() const;

  float ComputeError(const EdgeSegment& edge_segment);
  float ComputeError(const Position& pos);
  float fitting_error() { return fitting_error_; }
  float get_angle() const;
  EdgeSegment edge_segment() const;

 public:
  static Line FitFromEdgeSegment(const EdgeSegment& edge_segment);

 public:
  EdgeSegment edge_segment_;

  float parameters_[2] = {0.0f, 0.0f};
  float fitting_error_ = 0.0f;
  bool is_parameter_of_x_ = false;

  int range[2] = {0, 0};
};

#endif