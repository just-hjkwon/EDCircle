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

Circle Arc::fitted_circle() const { return fitted_circle_; }

float Arc::length() const { return length_; }
