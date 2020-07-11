#include "EDLine.h"

EDLine::EDLine() : EDPF() {}

void EDLine::DetectLine(GrayImage &image) {
  DetectEdge(image);
  ExtractLine();
}

float EDLine::ComputeFitError(const Line &line, const EdgeSegment &segment) {
  float error = 0.0f;

  for (const auto &s : segment) {
    error += ComputeFitError(line, s.first);
  }

  error /= float(segment.size());

  return error;
}

float EDLine::ComputeFitError(const Line &line, const Position &pos) {
  float error = 0.0f;

  if (line.is_parameter_of_x == true) {
    error = abs(line.parameters[0] * pos.x - pos.y + line.parameters[1]) /
            sqrt(line.parameters[0] * line.parameters[0] + 1);
  } else {
    error = abs(-pos.x + line.parameters[0] * pos.y + line.parameters[1]) /
            sqrt(line.parameters[0] * line.parameters[0] + 1);
  }

  return error;
}

Line EDLine::FitLine(const EdgeSegment &segment) {
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

  Line line;

  if (inv_denominator_by_x >= inv_denominator_by_y) {
    line.is_parameter_of_x = true;

    float parameter_a =
        ((n * sum_xy) + (-sum_x * sum_y)) / inv_denominator_by_x;
    float parameter_b =
        ((-sum_x * sum_xy) + (sum_xx * sum_y)) / inv_denominator_by_x;

    line.parameters[0] = parameter_a;
    line.parameters[1] = parameter_b;
  } else {
    line.is_parameter_of_x = false;

    float parameter_a =
        ((n * sum_xy) + (-sum_y * sum_x)) / inv_denominator_by_y;
    float parameter_b =
        ((-sum_y * sum_xy) + (sum_yy * sum_x)) / inv_denominator_by_y;

    line.parameters[0] = parameter_a;
    line.parameters[1] = parameter_b;
  }

  float error = ComputeFitError(line, segment);
  line.fitting_error = error;

  return line;
}

void EDLine::ExtractLine() {
  minimum_line_length_ = int(
      round(-4.0f * log(sqrt(float(width_) * float(height_))) / log(0.125f)));

  for (auto &segment : edge_segments_) {
    std::vector<Line> line_segments = ExtractLineSegments(segment);
  }
}

std::vector<Line> EDLine::ExtractLineSegments(const EdgeSegment segment) {
  std::vector<Line> line_segments;

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

    Line line = FitLine(_segment);

    while (line.fitting_error > 1.0f) {
      if (line_segment_end == segment.end()) {
        break;
      }

      _segment.pop_front();
      _segment.push_back(*line_segment_end);
      line_segment_end++;

      line = FitLine(_segment);
    }

    if (line.fitting_error > 1.0f) {
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
