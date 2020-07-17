#include "ed_line.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/highgui.hpp>

#include "util.h"

EDLine::EDLine() : EDPF() {
  anchor_threshold_ = 2.0f;
  aligned_degree_treshold_ = M_PI / 8.0f;
  precision_ = 1.0f / 8.0f;
}

void EDLine::DetectLine(GrayImage &image) {
  PrepareEdgeMap(image);

  STOPWATCHSTART(verbose_)
  ExtractLine();
  STOPWATCHSTOP(verbose_, "EDLine::DetectLine - ")
}

std::list<Line> EDLine::lines() { return lines_; }

bool EDLine::isValidLineSegment(const Line &line) {
  float line_angle = line.get_angle();

  if (line_angle >= M_PI / 2.0f) {
    line_angle -= M_PI;
  } else if (line_angle <= -M_PI / 2.0f) {
    line_angle += M_PI;
  }

  EdgeSegment edge_segment = line.edge_segment();

  int aligned_edge_count = 0;
  int segment_length = int(edge_segment.size());

  for (const auto &e : edge_segment) {
    std::size_t offset = get_offset(e.position);

    int gx = gx_.buffer()[offset];
    int gy = gy_.buffer()[offset];

    if (gy < 0) {
      gy *= -1;
    } else {
      gx *= -1;
    }

    float edge_degree = atan2(float(gx), float(gy));
    float degree_difference = abs(line_angle - edge_degree);
    if (line_angle >= 0.0f && edge_degree < 0.0f) {
      degree_difference = std::min(degree_difference,
                                   abs(line_angle - float(edge_degree + M_PI)));
    } else if (line_angle < 0.0f && edge_degree >= 0.0f) {
      degree_difference = std::min(degree_difference,
                                   abs(line_angle - float(edge_degree - M_PI)));
    }

    if (degree_difference < aligned_degree_treshold_) {
      aligned_edge_count++;
    }
  }

  float nfa = getSegmentNFA(segment_length, aligned_edge_count);

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

void EDLine::ExtractLine() {
  minimum_line_length_ = int(
      round(-4.0f * log(sqrt(float(width_) * float(height_))) / log(0.125f)));

  std::list<Line> lines;

  for (auto &edge_segment : edge_segments_) {
    std::vector<Line> line_segments = ExtractLineSegments(edge_segment);

    for (auto &line : line_segments) {
      if (isValidLineSegment(line) == true) {
        lines.push_back(line);
      }
    }
  }

  lines_ = lines;
}

std::vector<Line> EDLine::ExtractLineSegments(const EdgeSegment &edge_segment) {
  std::list<Line> lines;

  EdgeSegment::const_iterator line_candidate_begin = edge_segment.begin();
  EdgeSegment::const_iterator line_candidate_end = edge_segment.begin();

  for (auto i = 0;
       i < minimum_line_length_ && line_candidate_end != edge_segment.end();
       ++i) {
    line_candidate_end++;
  }

  EdgeSegment _segment;
  _segment.insert(_segment.end(), line_candidate_begin, line_candidate_end);

  while (true) {
    if (_segment.size() < minimum_line_length_) {
      break;
    }

    Line new_line = Line::FitFromEdgeSegment(_segment);

    while (new_line.fitting_error() > 1.0f) {
      if (line_candidate_end == edge_segment.end()) {
        break;
      }

      _segment.pop_front();
      _segment.push_back(*line_candidate_end);
      line_candidate_end++;

      new_line = Line::FitFromEdgeSegment(_segment);
    }

    if (new_line.fitting_error() > 1.0f) {
      break;
    }

    while (line_candidate_end != edge_segment.end()) {
      float error = new_line.ComputeError(line_candidate_end->position);

      if (error > 1.0) {
        break;
      }

      _segment.push_back(*line_candidate_end);
      line_candidate_end++;
    }

    new_line = Line::FitFromEdgeSegment(_segment);

    lines.push_back(new_line);

    line_candidate_begin = line_candidate_end;

    while (line_candidate_begin != line_candidate_end) {
      line_candidate_begin++;
    }

    for (auto i = 0;
         i < minimum_line_length_ && line_candidate_end != edge_segment.end();
         ++i) {
      line_candidate_end++;
    }
    _segment.clear();
    _segment.insert(_segment.end(), line_candidate_begin, line_candidate_end);
  }

  std::vector<Line> line_vector;
  line_vector.insert(line_vector.end(), lines.begin(), lines.end());

  return line_vector;
}
