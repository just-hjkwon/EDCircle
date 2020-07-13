#include "line.h"

#include <algorithm>
#include <opencv2/imgproc.hpp>

Line::Line(float a, float b, float fitting_error, bool is_parameter_of_x,
           EdgeSegment edge_segment) {
  parameters_[0] = a;
  parameters_[1] = b;

  fitting_error_ = fitting_error;

  is_parameter_of_x_ = is_parameter_of_x;
  edge_segment_ = edge_segment;

  range[0] = INT_MAX;
  range[1] = -INT_MAX;

  if (is_parameter_of_x_ == true) {
    for (auto edge : edge_segment) {
      range[0] = std::min(range[0], edge.first.x);
      range[1] = std::max(range[1], edge.first.x);
    }
  } else {
    for (auto edge : edge_segment) {
      range[0] = std::min(range[0], edge.first.y);
      range[1] = std::max(range[1], edge.first.y);
    }
  }

  Position p0 = begin();
  Position p1 = end();

  length_ = sqrt((p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y));
}

Position Line::begin() const { return edge_segment_.front().first; }

Position Line::end() const { return edge_segment_.back().first; }

Position Line::vector() const {
  return Position(edge_segment_.back().first.x - edge_segment_.front().first.x,
                  edge_segment_.back().first.y - edge_segment_.front().first.y);
}

float Line::length() const { return length_; }

float Line::ComputeError(const EdgeSegment& edge_segment) {
  float error = 0.0f;

  for (const auto& edge : edge_segment) {
    error += ComputeError(edge.first);
  }

  error /= float(edge_segment.size());

  return error;
}

float Line::ComputeError(const Position& pos) {
  float error = 0.0f;
  float x = float(pos.x);
  float y = float(pos.y);

  if (is_parameter_of_x_ == true) {
    error = abs(parameters_[0] * x - y + parameters_[1]) /
            sqrt(parameters_[0] * parameters_[0] + 1.0f);
  } else {
    error = abs(-x + parameters_[0] * y + parameters_[1]) /
            sqrt(parameters_[0] * parameters_[0] + 1.0f);
  }

  return error;
}

float Line::get_angle() const {
  if (is_parameter_of_x_ == true) {
    return atan2(parameters_[0], 1.0f);
  } else {
    return atan2(1.0f, parameters_[0]);
  }
}

EdgeSegment Line::edge_segment() const { return edge_segment_; }

void Line::Draw(cv::Mat& image, cv::Scalar color) const {
  Position b = begin();
  Position e = end();

  cv::line(image, cv::Point(b.x, b.y), cv::Point(e.x, e.y), color);
}

Line Line::FitFromEdgeSegment(const EdgeSegment& edge_segment) {
  float sum_xx = 0.0f;
  float sum_yy = 0.0f;
  float sum_xy = 0.0f;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float n = float(edge_segment.size());

  for (const auto& s : edge_segment) {
    sum_xx += float(s.first.x * s.first.x);
    sum_yy += float(s.first.y * s.first.y);
    sum_xy += float(s.first.x * s.first.y);
    sum_x += float(s.first.x);
    sum_y += float(s.first.y);
  }

  float inv_denominator_by_x = (sum_xx * n) - (sum_x * sum_x);
  float inv_denominator_by_y = (sum_yy * n) - (sum_y * sum_y);

  bool is_parameter_of_x = true;
  float a = 0.0f;
  float b = 0.0f;

  if (inv_denominator_by_x >= inv_denominator_by_y) {
    is_parameter_of_x = true;

    a = ((n * sum_xy) + (-sum_x * sum_y)) / inv_denominator_by_x;
    b = ((-sum_x * sum_xy) + (sum_xx * sum_y)) / inv_denominator_by_x;
  } else {
    is_parameter_of_x = false;

    a = ((n * sum_xy) + (-sum_y * sum_x)) / inv_denominator_by_y;
    b = ((-sum_y * sum_xy) + (sum_yy * sum_x)) / inv_denominator_by_y;
  }

  Line line(a, b, 0.0f, is_parameter_of_x, edge_segment);

  float error = line.ComputeError(edge_segment);
  line.fitting_error_ = error;

  return line;
}
