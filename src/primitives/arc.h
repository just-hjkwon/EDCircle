#ifndef PRIMITIVES__ARC_H_
#define PRIMITIVES__ARC_H_

#include <opencv2/core.hpp>
#include <vector>

#include "../types.h"
#include "circle.h"
#include "line.h"

class Arc {
 public:
  Arc(const std::vector<Line> &lines);
  Arc(const std::vector<Line> &lines, const Circle &fitted_circle);

 public:
  float ComputeNearestDistanceWithEndPoint(const Arc &other) const;
  void Draw(cv::Mat &image, cv::Scalar color);

  Circle fitted_circle() const;
  float length() const;
  std::vector<Line> lines() { return lines_; }

 protected:
  Circle fitted_circle_;
  std::vector<Line> lines_;
  float length_;
};

#endif