#ifndef PRIMITIVES__ELLIPSE_H_
#define PRIMITIVES__ELLIPSE_H_

#include <opencv2/core.hpp>
#include <vector>

#include "../types.h"
#include "line.h"

class Ellipse {
 public:
  Ellipse(float a, float b, float c, float d, float e, float f,
          float fitting_error);

 public:
  float get_circumference() const;
  PositionF get_center() const;
  Position get_positionAt(float degree) const;
  float fitting_error() { return fitting_error_; }
  void Draw(cv::Mat &image, cv::Scalar color) const;

  float angle() const;
  float major_length() const;
  float minor_length() const;

 public:
  static Ellipse FitFromEdgeSegment(const EdgeSegment &edge_segment);
  static Ellipse FitFromLines(const std::vector<Line> &lines);

 protected:
  float ComputeError(Position position);

 protected:
  float parameters_[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float cx_ = 0.0f;
  float cy_ = 0.0f;
  float axis_lengths_[2] = {0.0f, 0.0f};
  float angle_ = 0.0f;

  float fitting_error_ = 0.0f;

  float cos_angle_ = 0.0f;
  float sin_angle_ = 0.0f;
};

#endif
