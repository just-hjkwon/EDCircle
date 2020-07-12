#ifndef PRIMITIVES__CIRCLE_H_
#define PRIMITIVES__CIRCLE_H_

#include <opencv2/core.hpp>
#include <vector>

#include "../types.h"
#include "line.h"

class Circle {
 public:
  Circle(float center_x, float center_y, float radius, float fitting_error_);

 public:
  void Draw(cv::Mat &image, cv::Scalar color);
  float fitting_error() { return fitting_error_; }
  PositionF get_center() { return PositionF(parameters_[0], parameters_[1]); }
  float get_radius() { return parameters_[2]; }

 public:
  static Circle FitFromEdgeSegment(const EdgeSegment &edge_segment);
  static Circle FitFromEdgeSegment(const std::vector<Line> &lines);

 public:
  float parameters_[3] = {0.0f, 0.0f, 0.0f};
  float fitting_error_ = 0.0f;
};

#endif
