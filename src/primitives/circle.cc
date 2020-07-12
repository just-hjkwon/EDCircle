#include "circle.h"

#include <opencv2/imgproc.hpp>

Circle::Circle(float center_x, float center_y, float radius,
               float fitting_error)
    : parameters_{center_x, center_y, radius}, fitting_error_(fitting_error) {}

void Circle::Draw(cv::Mat& image, cv::Scalar color) {
  cv::circle(image, cv::Point(parameters_[0], parameters_[1]), parameters_[2],
             color);
}

Circle Circle::FitFromEdgeSegment(const EdgeSegment& edge_segment) {
  float mean_x = 0.0f;
  float mean_y = 0.0f;

  float sum_uu = 0.0f;
  float sum_uv = 0.0f;
  float sum_vv = 0.0f;
  float sum_uuu = 0.0f;
  float sum_uvv = 0.0f;
  float sum_vvv = 0.0f;
  float sum_vuu = 0.0f;

  for (auto e : edge_segment) {
    mean_x += float(e.first.x);
    mean_y += float(e.first.y);
  }

  mean_x /= float(edge_segment.size());
  mean_y /= float(edge_segment.size());

  for (auto e : edge_segment) {
    float u = (float(e.first.x) - mean_x);
    float v = (float(e.first.y) - mean_y);

    sum_uu += u * u;
    sum_vv += v * v;
    sum_uv += u * v;
    sum_uuu += u * u * u;
    sum_uvv += u * v * v;
    sum_vvv += v * v * v;
    sum_vuu += v * u * u;
  }

  float inv_denominator = (sum_uu * sum_vv) - (sum_uv * sum_uv);

  float center_x = ((sum_vv * ((sum_uuu + sum_uvv) / 2.0f)) / inv_denominator) +
                   ((-sum_uv * ((sum_vvv + sum_vuu) / 2.0f)) / inv_denominator);
  float center_y =
      ((-sum_uv * ((sum_uuu + sum_uvv) / 2.0f)) / inv_denominator) +
      ((sum_uu * ((sum_vvv + sum_vuu) / 2.0f)) / inv_denominator);

  float radius = (center_x * center_x) + (center_y * center_y) +
                 ((sum_uu + sum_vv) / float(edge_segment.size()));

  center_x += mean_x;
  center_y += mean_y;

  radius = sqrt(radius);

  float error = 0.0f;

  for (auto e : edge_segment) {
    float x = float(e.first.x);
    float y = float(e.first.y);
    error += abs(sqrt(((x - center_x) * (x - center_x)) +
                      ((y - center_y) * (y - center_y))) -
                 radius);
  }
  error /= float(edge_segment.size());

  return Circle(center_x, center_y, radius, error);
}

Circle Circle::FitFromEdgeSegment(const std::vector<Line>& lines) {
  EdgeSegment whole_edge_segment;

  for (const auto& line : lines) {
    EdgeSegment edge_segment = line.edge_segment();
    whole_edge_segment.insert(whole_edge_segment.end(), edge_segment.begin(),
                              edge_segment.end());
  }

  return FitFromEdgeSegment(whole_edge_segment);
}
