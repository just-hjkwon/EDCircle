#include "ed_line.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/highgui.hpp>

EDLine::EDLine() : EDPF() {
  anchor_threshold_ = 2.0f;
  aligned_degree_treshold_ = M_PI / 8.0f;
  precision_ = 1.0f / 8.0f;
}

void EDLine::DetectLine(GrayImage &image) {
  DetectEdge(image);
  ExtractLine();
}

float EDLine::ComputeFitError(const LineSegment &line,
                              const EdgeSegment &segment) {
  float error = 0.0f;

  for (const auto &s : segment) {
    error += ComputeFitError(line, s.first);
  }

  error /= float(segment.size());

  return error;
}

float EDLine::ComputeFitError(const LineSegment &line, const Position &pos) {
  float error = 0.0f;

  if (line.is_parameter_of_x == true) {
    error = abs(line.parameters_[0] * pos.x - pos.y + line.parameters_[1]) /
            sqrt(line.parameters_[0] * line.parameters_[0] + 1);
  } else {
    error = abs(-pos.x + line.parameters_[0] * pos.y + line.parameters_[1]) /
            sqrt(line.parameters_[0] * line.parameters_[0] + 1);
  }

  return error;
}

bool EDLine::isValidLineSegment(LineSegment line) {
  float line_degree = 0.0f;

  if (line.is_parameter_of_x == true) {
    line_degree = atan2(line.parameters_[0], 1.0f);
  } else {
    line_degree = atan2(1.0f, line.parameters_[0]);
  }

  if (line_degree >= M_PI / 2.0f) {
    line_degree -= M_PI;
  } else if (line_degree <= -M_PI / 2.0f) {
    line_degree += M_PI;
  }

  int aligned_edge_count = 0;

  for (const auto e : line.edges) {
    std::size_t offset = get_offset(e.first);

    float gx = x_gradient_->buffer()[offset];
    float gy = y_gradient_->buffer()[offset];

    if (gy < 0) {
      gy *= -1.0f;
    } else {
      gx *= -1.0f;
    }

    float edge_degree = atan2(gx, gy);
    float degree_difference = abs(line_degree - edge_degree);
    if (line_degree >= 0.0f && edge_degree < 0.0f) {
      degree_difference =
          std::min(degree_difference, abs(line_degree - float(edge_degree + M_PI)));
    } else if (line_degree < 0.0f && edge_degree >= 0.0f) {
      degree_difference = std::min(degree_difference,
                                abs(line_degree - float(edge_degree - M_PI)));
    }

    if (degree_difference < aligned_degree_treshold_) {
      aligned_edge_count++;
    }
  }

  float nfa = getSegmentNFA(int(line.edges.size()), aligned_edge_count);

  if (nfa <= 1.0f) {
    return true;
  } else {
    return false;
  }
}

float EDLine::getSegmentNFA(int segment_length, int aligned_count) {
  const float N = float(width_ * height_ * width_ * height_);

  float factorial = 0.0f;
  for (auto i = aligned_count; i <= segment_length; ++i) {
    int diff1 = segment_length - i;
    int diff2 = segment_length - diff1;
    int bigger_diff = std::max(diff1, diff2);
    int smaller_diff = std::min(diff1, diff2);

    float _f = 1.0f;

    for (auto n = bigger_diff + 1; n <= segment_length; ++n) {
      _f *= float(n);
    }
    for (auto n = 2; n <= smaller_diff; ++n) {
      _f /= float(n);
    }

    _f *= pow(precision_, float(i)) *
          pow(1.0f - precision_, float(segment_length - i));
    factorial += _f;
  }

  return N * factorial;
}

LineSegment EDLine::FitLine(const EdgeSegment &segment) {
  float sum_xx = 0.0f;
  float sum_yy = 0.0f;
  float sum_xy = 0.0f;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float n = float(segment.size());

  for (const auto &s : segment) {
    sum_xx += float(s.first.x * s.first.x);
    sum_yy += float(s.first.y * s.first.y);
    sum_xy += float(s.first.x * s.first.y);
    sum_x += float(s.first.x);
    sum_y += float(s.first.y);
  }

  float inv_denominator_by_x = (sum_xx * n) - (sum_x * sum_x);
  float inv_denominator_by_y = (sum_yy * n) - (sum_y * sum_y);

  LineSegment line;

  if (inv_denominator_by_x >= inv_denominator_by_y) {
    line.is_parameter_of_x = true;

    float parameter_a =
        ((n * sum_xy) + (-sum_x * sum_y)) / inv_denominator_by_x;
    float parameter_b =
        ((-sum_x * sum_xy) + (sum_xx * sum_y)) / inv_denominator_by_x;

    line.parameters_[0] = parameter_a;
    line.parameters_[1] = parameter_b;
  } else {
    line.is_parameter_of_x = false;

    float parameter_a =
        ((n * sum_xy) + (-sum_y * sum_x)) / inv_denominator_by_y;
    float parameter_b =
        ((-sum_y * sum_xy) + (sum_yy * sum_x)) / inv_denominator_by_y;

    line.parameters_[0] = parameter_a;
    line.parameters_[1] = parameter_b;
  }

  line.edges = segment;

  float error = ComputeFitError(line, segment);
  line.fitting_error_ = error;

  return line;
}

void EDLine::ExtractLine() {
  minimum_line_length_ = int(
      round(-4.0f * log(sqrt(float(width_) * float(height_))) / log(0.125f)));

  line_segments_.clear();
  line_segments_.reserve(width_ * height_);

  for (auto &edge : edge_segments_) {
    std::vector<LineSegment> line_segments = ExtractLineSegments(edge);

    for (auto &line : line_segments) {
      if (isValidLineSegment(line) == true) {
        line_segments_.push_back(line);
      }
    }
  }

  line_segments_.shrink_to_fit();
}

std::vector<LineSegment> EDLine::ExtractLineSegments(
    const EdgeSegment segment) {
  std::vector<LineSegment> line_segments;

  EdgeSegment::const_iterator line_segment_begin = segment.begin();
  EdgeSegment::const_iterator line_segment_end = segment.begin();

  for (auto i = 0;
       i < minimum_line_length_ && line_segment_end != segment.end(); ++i) {
    line_segment_end++;
  }

  EdgeSegment _segment;
  _segment.insert(_segment.end(), line_segment_begin, line_segment_end);

  while (true) {
    if (_segment.size() < minimum_line_length_) {
      break;
    }

    LineSegment line = FitLine(_segment);

    while (line.fitting_error_ > 1.0f) {
      if (line_segment_end == segment.end()) {
        break;
      }

      _segment.pop_front();
      _segment.push_back(*line_segment_end);
      line_segment_end++;

      line = FitLine(_segment);
    }

    if (line.fitting_error_ > 1.0f) {
      break;
    }

    while (line_segment_end != segment.end()) {
      float error = ComputeFitError(line, line_segment_end->first);

      if (error > 1.0) {
        break;
      }

      _segment.push_back(*line_segment_end);
      line_segment_end++;
    }

    line = FitLine(_segment);
    line_segments.push_back(line);

    line_segment_begin = line_segment_end;

    while (line_segment_begin != line_segment_end) {
      line_segment_begin++;
    }

    for (auto i = 0;
         i < minimum_line_length_ && line_segment_end != segment.end(); ++i) {
      line_segment_end++;
    }
    _segment.clear();
    _segment.insert(_segment.end(), line_segment_begin, line_segment_end);
  }

  return line_segments;
}
