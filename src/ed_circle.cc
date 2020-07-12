#include "ed_circle.h"

#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

#include "primitives/line.h"

EDCircle::EDCircle() {
  circle_fitting_error_threshold_ = 1.5f;
  ellipse_fitting_error_threshold_ = 1.5f;

  arc_line_angle_thresholds_[0] = 6.0f / 180.0f * M_PI;
  arc_line_angle_thresholds_[1] = 60.0f / 180.0f * M_PI;
}

void EDCircle::DetectCircle(GrayImage& image) {
  DetectEdge(image);

  DetectCircleAndEllipseFromClosedEdgeSegment();
  ExtractArcs();
}

bool EDCircle::isClosedEdgeSegment(const EdgeSegment& edge_segment) {
  auto first_edge = edge_segment.front();
  auto last_edge = edge_segment.back();

  int x_diff = abs(first_edge.first.x - last_edge.first.x);
  int y_diff = abs(first_edge.first.y - last_edge.first.y);

  if (x_diff <= 1 && y_diff <= 1) {
    return true;
  } else {
    return false;
  }
}

void EDCircle::DetectCircleAndEllipseFromClosedEdgeSegment() {
  circles_.clear();
  ellipses_.clear();

  for (auto edge_it = edge_segments_.begin();
       edge_it != edge_segments_.end();) {
    if (isClosedEdgeSegment(*edge_it) == true) {
      Circle circle = Circle::FitFromEdgeSegment(*edge_it);

      if (circle.fitting_error() < circle_fitting_error_threshold_) {
        circles_.push_back(circle);
        edge_it = edge_segments_.erase(edge_it);
        continue;
      }

      Ellipse ellipse = Ellipse::FitFromEdgeSegment(*edge_it);

      if (ellipse.fitting_error() < ellipse_fitting_error_threshold_) {
        ellipses_.push_back(ellipse);
        edge_it = edge_segments_.erase(edge_it);
        continue;
      }
    }
    edge_it++;
  }
}

std::vector<Arc> EDCircle::ExtractArcFromLines(const std::vector<Line>& lines) {
  std::vector<Arc> arcs;

  std::vector<float> lengths;
  std::vector<float> angles;
  std::vector<unsigned char> turn_directions;

  lengths.reserve(lines.size());
  angles.reserve(lines.size());
  turn_directions.reserve(lines.size());

  if (lines.size() < 3) {
    return arcs;
  }

  lengths.push_back(lines[0].get_length());

  for (auto i = 1; i < lines.size(); ++i) {
    lengths.push_back(lines[i].get_length());

    Position prev_vector = lines[i - 1].vector();
    Position cur_vector = lines[i].vector();

    float dot_product =
        float(prev_vector.x * cur_vector.x + prev_vector.y * cur_vector.y);
    float angle = acos(dot_product / (lengths[i - 1] * lengths[i]));

    float cross_product = asin(
        float(prev_vector.x * cur_vector.y - prev_vector.y * cur_vector.x) /
        (lengths[i - 1] * lengths[i]));

    angles.push_back(angle);

    if (cross_product >= 0.0f) {
      turn_directions.push_back(-1);
    } else {
      turn_directions.push_back(1);
    }
  }

  unsigned char current_turn_direction = turn_directions[0];
  auto candidnate_begin = lines.begin();
  auto candidnate_end = lines.begin() + 1;

  int info_index = 0;
  while (candidnate_begin != lines.end()) {
    if (candidnate_end != lines.end() &&
        arc_line_angle_thresholds_[0] < angles[info_index] &&
        angles[info_index] < arc_line_angle_thresholds_[1] &&
        current_turn_direction == turn_directions[info_index]) {
      candidnate_end++;
      info_index++;
      continue;
    }

    if (candidnate_end - candidnate_begin < 3) {
      if (candidnate_end == lines.end()) {
        break;
      }

      current_turn_direction = turn_directions[info_index];
      candidnate_begin = candidnate_end;
      candidnate_end++;
      info_index++;
      continue;
    }

    std::vector<Line> arc_lines;
    arc_lines.insert(arc_lines.end(), candidnate_begin, candidnate_end);
    Arc arc(arc_lines);
    arcs.push_back(arc);

    if (candidnate_end == lines.end()) {
      break;
    }

    candidnate_begin = candidnate_end;
    current_turn_direction = turn_directions[info_index];
    candidnate_end++;
    info_index++;
  }

  return arcs;
}

void EDCircle::ExtractArcs() {
  minimum_line_length_ = int(
      round(-4.0f * log(sqrt(float(width_) * float(height_))) / log(0.125f)));

  for (auto& edge : edge_segments_) {
    std::vector<Line> lines = ExtractLineSegments(edge);

    std::vector<Arc> arcs = ExtractArcFromLines(lines);
  }
}
