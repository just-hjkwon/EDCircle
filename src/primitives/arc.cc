#include "arc.h"

Arc::Arc(const std::vector<Line>& lines)
    : lines_(lines), fitted_circle_(Circle::FitFromEdgeSegment(lines)) {
  length_ = 0.0f;
  for (const auto& line : lines_) {
    length_ += line.length();
  }
}

Arc::Arc(const std::vector<Line>& lines, const Circle& fitted_circle)
    : lines_(lines), fitted_circle_(fitted_circle) {
  length_ = 0.0f;
  for (const auto& line : lines_) {
    length_ += line.length();
  }
}

float Arc::ComputeNearestDistanceWithEndPoint(const Arc& other) const {
  Position p0 = lines_.front().begin();
  Position p1 = lines_.back().end();

  Position o_p0 = other.lines_.front().begin();
  Position o_p1 = other.lines_.back().end();

  float nearest_distance = FLT_MAX;

  std::min(nearest_distance, p0.DistanceWith(o_p0));
  std::min(nearest_distance, p0.DistanceWith(o_p1));
  std::min(nearest_distance, p1.DistanceWith(o_p0));
  std::min(nearest_distance, p1.DistanceWith(o_p1));

  return nearest_distance;
}

void Arc::Draw(cv::Mat& image, cv::Scalar color) {
  for (auto line : lines_) {
    line.Draw(image, color);
  }
}

Circle Arc::fitted_circle() const { return fitted_circle_; }

float Arc::length() const { return length_; }
