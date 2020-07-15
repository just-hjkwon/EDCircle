#include "ellipse.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/imgproc.hpp>

Ellipse::Ellipse(float a, float b, float c, float d, float e, float f,
                 float fitting_error)
    : parameters_{a, b, c, d, e, f}, fitting_error_(fitting_error) {
  cx_ = (2 * c * d - b * e) / (b * b - 4 * a * c);
  cy_ = (2 * a * e - b * d) / (b * b - 4 * a * c);

  if (b != 0.0f) {
    angle_ = atan(((c - a - sqrt((a - c) * (a - c) + b * b))) / b);
  } else if (a < c) {
    angle_ = 0.0f;
  } else {
    angle_ = M_PI / 2.0f;
  }

  float term0 =
      2.0f * (a * e * e + c * d * d - b * d * e + (b * b - 4.0f * a * c) * f);
  float term1 = (a + c);
  float term2 = sqrt((a - c) * (a - c) + b * b);
  float term4 = (b * b - 4.0f * a * c);

  axis_lengths_[0] = (-1.0f * sqrt(term0 * (term1 + term2))) / term4;
  axis_lengths_[1] = (-1.0f * sqrt(term0 * (term1 - term2))) / term4;

  cos_angle_ = cos(angle_);
  sin_angle_ = sin(angle_);
}

float Ellipse::get_circumference() const {
  return 2.0f *
         sqrt((axis_lengths_[0] * axis_lengths_[0] +
               axis_lengths_[1] * axis_lengths_[1]) /
              2.0f) *
         M_PI;
}

PositionF Ellipse::get_center() const { return PositionF(cx_, cy_); }

Position Ellipse::get_positionAt(float degree) const {
  float angle = degree / 180.0f * M_PI;

  float cos_angle = cos(angle);
  float sin_angle = sin(angle);

  float new_aspect_x = cos(angle);
  float new_aspect_y = sin(angle);
  angle = atan2(new_aspect_y, new_aspect_x);

  float ideal_x = cx_ + axis_lengths_[0] * cos_angle * cos_angle_ -
                  axis_lengths_[1] * sin_angle * sin_angle_;
  float ideal_y = cy_ + axis_lengths_[0] * cos_angle * sin_angle_ +
                  axis_lengths_[1] * sin_angle * cos_angle_;

  return Position(int(ideal_x + 0.5f), int(ideal_y + 0.5f));
}

void Ellipse::Draw(cv::Mat& image, cv::Scalar color) const {
  cv::ellipse(image, cv::Point2f(cx_, cy_),
              cv::Size(axis_lengths_[0], axis_lengths_[1]),
              angle_ / M_PI * 180.0f, 0.0, 360.0, color, 2);
}

Ellipse Ellipse::FitFromEdgeSegment(const EdgeSegment& edge_segment) {
  std::vector<cv::Point2f> points;
  for (const auto& e : edge_segment) {
    points.push_back(cv::Point2f(e.first.x, e.first.y));
  }

  cv::RotatedRect rect = cv::fitEllipseDirect(points);

  float major_length = 0.0f;
  float minor_length = 0.0f;
  float angle = 0.0f;

  if (rect.size.width >= rect.size.height) {
    major_length = rect.size.width / 2.0f;
    minor_length = rect.size.height / 2.0f;
    angle = rect.angle;
  } else {
    major_length = rect.size.height / 2.0f;
    minor_length = rect.size.width / 2.0f;
    angle = (90.0f + rect.angle);
  }

  angle = angle / 180.0f * M_PI;

  float major_length2 = major_length * major_length;
  float minor_length2 = minor_length * minor_length;
  float center_x = rect.center.x;
  float center_y = rect.center.y;

  float cos_angle = cos(angle);
  float cos_angle2 = cos_angle * cos_angle;
  float sin_angle = sin(angle);
  float sin_angle2 = sin_angle * sin_angle;

  float a = major_length2 * sin_angle2 + minor_length2 * cos_angle2;
  float b = 2.0f * (minor_length2 - major_length2) * sin_angle * cos_angle;
  float c = major_length2 * cos_angle2 + minor_length2 * sin_angle2;
  float d = -2.0f * a * center_x + -1.0f * b * center_y;
  float e = -1.0f * b * center_x + -2.0f * c * center_y;
  float f = a * center_x * center_x + b * center_x * center_y +
            c * center_y * center_y - major_length2 * minor_length2;

  float parameters[6] = {a, b, c, d, e, f};

  Ellipse ellipse(a, b, c, d, e, f, 0.0f);

  float error = 0.0f;

  for (const auto& edge : edge_segment) {
    error += ellipse.ComputeError(edge.first);
  }
  error /= float(edge_segment.size());

  ellipse.fitting_error_ = error;

  return ellipse;
}

Ellipse Ellipse::FitFromEdgeSegment(const std::vector<Line>& lines) {
  EdgeSegment whole_edge_segment;

  for (const auto& line : lines) {
    EdgeSegment edge_segment = line.edge_segment();
    whole_edge_segment.insert(whole_edge_segment.end(), edge_segment.begin(),
                              edge_segment.end());
  }

  return FitFromEdgeSegment(whole_edge_segment);
}

float Ellipse::ComputeError(Position pos) {
  float x = float(pos.x);
  float y = float(pos.y);

  float degree = atan2(y - cy_, x - cx_) - angle_;

  float new_aspect_x = cos(degree) / axis_lengths_[0];
  float new_aspect_y = sin(degree) / axis_lengths_[1];
  degree = atan2(new_aspect_y, new_aspect_x);

  float ideal_x = cx_ + axis_lengths_[0] * cos(degree) * cos_angle_ -
                  axis_lengths_[1] * sin(degree) * sin_angle_;
  float ideal_y = cy_ + axis_lengths_[0] * cos(degree) * sin_angle_ +
                  axis_lengths_[1] * sin(degree) * cos_angle_;

  float error =
      sqrt((ideal_x - x) * (ideal_x - x) + (ideal_y - y) * (ideal_y - y));

  return error;
}
