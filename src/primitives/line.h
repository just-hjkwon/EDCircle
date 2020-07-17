#ifndef PRIMITIVES__LINE_H_
#define PRIMITIVES__LINE_H_

#include <opencv2/core.hpp>

#include "../types.h"
#include "edge_segment.h"

class Line {
 public:
  Line(float a, float b, float fitting_error, bool is_parameter_of_x,
       EdgeSegment edge_segment_);

 public:
  Position begin() const;
  Position end() const;
  Position vector() const;

  float length() const;

  float ComputeError(const EdgeSegment& edge_segment);
  float ComputeError(const Position& position);
  float fitting_error() { return fitting_error_; }
  float get_angle() const;
  EdgeSegment edge_segment() const;

  void Draw(cv::Mat& image, cv::Scalar color) const;

 public:
  static Line FitFromEdgeSegment(const EdgeSegment& edge_segment);

 protected:
  EdgeSegment edge_segment_;

  float parameters_[2] = {0.0f, 0.0f};
  float fitting_error_ = 0.0f;
  bool is_parameter_of_x_ = false;
  float length_ = 0.0f;
};

#endif