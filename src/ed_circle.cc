#include "ed_circle.h"

#include <chrono>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

#include "primitives/circle.h"
#include "primitives/line.h"

EDCircle::EDCircle() {
  circle_fitting_error_threshold_ = 1.5f;
  ellipse_fitting_error_threshold_ = 1.5f;

  arc_line_angle_thresholds_[0] = 6.0f / 180.0f * M_PI;
  arc_line_angle_thresholds_[1] = 60.0f / 180.0f * M_PI;
}

void EDCircle::DetectCircle(GrayImage& image) {
  image_ = std::make_shared<GrayImage>(image.width(), image.height(),
                                       image.buffer());

  std::chrono::system_clock::time_point start;
  std::chrono::duration<double> sec;

  start = std::chrono::system_clock::now();
  DetectEdge(image);
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDCircle::DetectEdge Elapsed time: " << sec.count() * 1000.0f
            << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  DetectCircleAndEllipseFromClosedEdgeSegment();
  sec = std::chrono::system_clock::now() - start;
  std::cout
      << "EDCircle::DetectCircleAndEllipseFromClosedEdgeSegment Elapsed time: "
      << sec.count() * 1000.0f << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  ExtractArcs();
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDCircle::ExtractArcs "
               "Elapsed time: "
            << sec.count() * 1000.0f << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  ExtendArcsAndDetectCircle();
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDCircle::ExtendArcsAndDetectCircle "
               "Elapsed time: "
            << sec.count() * 1000.0f << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  ExtendArcsAndDetectEllipse();
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDCircle::ExtendArcsAndDetectEllipse "
               "Elapsed time: "
            << sec.count() * 1000.0f << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  ValidateCircleAndEllipse();
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDCircle::ValidateCircleAndEllipse "
               "Elapsed time: "
            << sec.count() * 1000.0f << " ms" << std::endl;
}

std::list<Circle> EDCircle::circles() { return circles_; }

std::list<Ellipse> EDCircle::ellipses() { return ellipses_; }

std::list<Arc> EDCircle::arcs() { return arcs_; }

std::list<Arc> EDCircle::extended_arcs() { return extended_arcs_; }

void EDCircle::DetectCircleAndEllipseFromClosedEdgeSegment() {
  circles_.clear();
  ellipses_.clear();

  for (auto edge_it = edge_segments_.begin();
       edge_it != edge_segments_.end();) {
    if (edge_it->isClosed() == true) {
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

std::vector<std::vector<Line>> EDCircle::ExtractArcCandidates(
    const std::vector<Line>& lines) {
  std::vector<std::vector<Line>> arc_candidates;

  std::vector<float> lengths;
  std::vector<float> angles;
  std::vector<unsigned char> turn_directions;

  lengths.reserve(lines.size());
  angles.reserve(lines.size());
  turn_directions.reserve(lines.size());

  if (lines.size() < 3) {
    return arc_candidates;
  }

  lengths.push_back(lines[0].length());

  for (auto i = 1; i < lines.size(); ++i) {
    lengths.push_back(lines[i].length());

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

      candidnate_begin++;
      candidnate_end = candidnate_begin + 1;
      info_index = std::distance(lines.begin(), candidnate_begin);

      continue;
    }

    std::vector<Line> candidate;
    candidate.insert(candidate.end(), candidnate_begin, candidnate_end);
    arc_candidates.push_back(candidate);

    if (candidnate_end == lines.end()) {
      break;
    }

    candidnate_begin = candidnate_end;
    current_turn_direction = turn_directions[info_index];
    candidnate_end++;
    info_index++;
  }

  return arc_candidates;
}

void EDCircle::ExtendArcsAndDetectCircle() {
  const float kThresholdRatio = 0.25f;

  std::list<Arc> candidates;
  candidates.insert(candidates.end(), arcs_.begin(), arcs_.end());
  candidates.sort(
      [](const Arc& a, const Arc& b) { return a.length() > b.length(); });

  std::list<Arc> extended_arcs;

  while (candidates.empty() != true) {
    Arc target_arc = candidates.front();

    PositionF target_center = target_arc.fitted_circle().get_center();
    float target_radius = target_arc.fitted_circle().get_radius();
    float threashold = target_radius * kThresholdRatio;

    candidates.pop_front();

    std::list<Arc> extended_candidates;

    for (auto& it = candidates.begin(); it != candidates.end();) {
      PositionF center = it->fitted_circle().get_center();
      float radius = it->fitted_circle().get_radius();
      float center_distance = target_center.DistanceWith(center);

      if (abs(radius - target_radius) <= threashold &&
          center_distance <= threashold) {
        extended_candidates.push_back(*it);
        it = candidates.erase(it);
        continue;
      }
      it++;
    }

    extended_candidates.sort([target_arc](const Arc& a, const Arc& b) {
      float distance_a = a.ComputeNearestDistanceWithEndPoint(target_arc);
      float distance_b = b.ComputeNearestDistanceWithEndPoint(target_arc);

      return distance_a < distance_b;
    });

    auto extended_lines = target_arc.lines();
    bool is_extended = false;

    for (auto it = extended_candidates.begin();
         it != extended_candidates.end();) {
      std::vector<Line> new_lines = extended_lines;
      std::vector<Line> candidate_lines = it->lines();
      new_lines.insert(new_lines.end(), candidate_lines.begin(),
                       candidate_lines.end());

      Circle circle = Circle::FitFromEdgeSegment(new_lines);

      if (circle.fitting_error() <= 1.5f) {
        extended_lines = new_lines;

        it = extended_candidates.erase(it);
        is_extended = true;

        continue;
      }

      it++;
    }

    if (is_extended == true) {
      Arc arc(extended_lines);

      float length = arc.length();
      float circumference = 2.0f * arc.fitted_circle().get_radius() * M_PI;

      if (length > circumference * 0.5f) {
        circles_.push_back(arc.fitted_circle());
      } else {
        extended_arcs.push_back(arc);
      }
    } else {
      Arc arc(extended_lines);

      if (arc.fitted_circle().fitting_error() <= 1.5f) {
        float length = arc.length();
        float circumference = 2.0f * arc.fitted_circle().get_radius() * M_PI;

        if (length > circumference * 0.5f) {
          circles_.push_back(arc.fitted_circle());
        } else {
          extended_arcs.push_back(arc);
        }
      }
    }

    candidates.insert(candidates.end(), extended_candidates.begin(),
                      extended_candidates.end());
  }

  extended_arcs_ = extended_arcs;
}

void EDCircle::ExtendArcsAndDetectEllipse() {
  const float kThresholdRatio = 0.5f;

  std::list<Arc> candidates;
  candidates.insert(candidates.end(), extended_arcs_.begin(),
                    extended_arcs_.end());
  candidates.sort(
      [](const Arc& a, const Arc& b) { return a.length() > b.length(); });

  std::list<Arc> extended_arcs;

  while (candidates.empty() != true) {
    Arc target_arc = candidates.front();

    PositionF target_center = target_arc.fitted_circle().get_center();
    float target_radius = target_arc.fitted_circle().get_radius();
    float threashold = target_radius * kThresholdRatio;

    candidates.pop_front();

    std::list<Arc> extended_candidates;

    for (auto& it = candidates.begin(); it != candidates.end();) {
      PositionF center = it->fitted_circle().get_center();
      float radius = it->fitted_circle().get_radius();
      float center_distance = target_center.DistanceWith(center);

      if (abs(radius - target_radius) <= threashold &&
          center_distance <= threashold) {
        extended_candidates.push_back(*it);
        it = candidates.erase(it);
        continue;
      }
      it++;
    }

    extended_candidates.sort([target_arc](const Arc& a, const Arc& b) {
      float distance_a = a.ComputeNearestDistanceWithEndPoint(target_arc);
      float distance_b = b.ComputeNearestDistanceWithEndPoint(target_arc);

      return distance_a < distance_b;
    });

    auto extended_lines = target_arc.lines();
    int extended_count = 0;

    for (auto it = extended_candidates.begin();
         it != extended_candidates.end();) {
      std::vector<Line> new_lines = extended_lines;
      std::vector<Line> candidate_lines = it->lines();
      new_lines.insert(new_lines.end(), candidate_lines.begin(),
                       candidate_lines.end());

      Ellipse ellipse = Ellipse::FitFromEdgeSegment(new_lines);

      if (ellipse.fitting_error() <= 1.5f) {
        extended_lines = new_lines;

        it = extended_candidates.erase(it);
        extended_count++;

        continue;
      }

      it++;
    }

    if (extended_count > 0) {
      Arc arc(extended_lines);

      Ellipse ellipse = Ellipse::FitFromEdgeSegment(extended_lines);

      float length = arc.length();
      float circumference = ellipse.get_circumference();

      if (length > circumference * 0.5f) {
        ellipses_.push_back(ellipse);
      } else {
        Arc arc(extended_lines);
        extended_arcs.push_back(arc);
      }
    } else {
      Arc arc(extended_lines);
      Ellipse ellipse = Ellipse::FitFromEdgeSegment(extended_lines);

      if (ellipse.fitting_error() <= 1.5f) {
        float length = arc.length();
        float circumference = 2.0f * arc.fitted_circle().get_radius() * M_PI;

        if (length > circumference * 0.5f) {
          ellipses_.push_back(ellipse);
        } else {
          extended_arcs.push_back(arc);
        }
      }
    }

    candidates.insert(candidates.end(), extended_candidates.begin(),
                      extended_candidates.end());
  }

  extended_arcs_ = extended_arcs;
}

void EDCircle::ValidateCircleAndEllipse() {
  std::list<Circle> circles;

  for (const auto& c : circles_) {
    if (isValidCircle(c) == true) {
      circles.push_back(c);
    }
  }

  circles_ = circles;

  std::list<Ellipse> ellipses;

  for (const auto& e : ellipses_) {
    if (isValidEllipse(e) == true) {
      ellipses.push_back(e);
    }
  }

  ellipses_ = ellipses;
}

bool EDCircle::isValidCircle(const Circle& circle) {
  float circumference = circle.get_circumference();
  float degree_step = 1.0f;

  std::vector<Position> positions;
  positions.reserve(int(ceil(circumference)));

  Position prev_p(-1, -1);
  for (float degree = 0.0f; degree < 360.0f; degree += degree_step) {
    Position p = circle.get_positionAt(degree);
    if (isValidPosition(p) == false) {
      continue;
    }

    if (p.x != prev_p.x && p.y != prev_p.y) {
      positions.push_back(p);
      prev_p = p;
    }
  }

  PositionF center = circle.get_center();

  int circumference_length = int(positions.size());
  int aligned_count = 0;

  unsigned char* buffer = image_->buffer();

  for (auto p : positions) {
    int offset = p.y * width_ + p.x;

    int p00 = int(buffer[offset]);
    int p01 = int(buffer[offset + 1]);
    int p10 = int(buffer[offset + width_]);
    int p11 = int(buffer[offset + width_ + 1]);

    float gx = (p01 - p00 + p11 - p10) / 2.0f;
    float gy = (p10 - p00 + p11 - p01) / 2.0f;

    PositionF point_vector(p.x - center.x, p.y - center.y);

    float tangent1 = atan2(gy, gx);
    float tangent2 = atan2(-gy, -gx);

    float level_line_angle = atan2(point_vector.y, point_vector.x);
    float angle_diff = std::min(abs(tangent1 - level_line_angle),
                                abs(tangent2 - level_line_angle));

    if (angle_diff <= M_PI / 8.0f) {
      aligned_count++;
    }
  }

  float nfa = getCircleNFA(circumference_length, aligned_count);

  if (nfa <= 1.0f) {
    return true;
  } else {
    return false;
  }
}

bool EDCircle::isValidEllipse(const Ellipse& ellipse) {
  float circumference = ellipse.get_circumference();
  float degree_step = 1.0;

  std::vector<Position> positions;
  positions.reserve(int(ceil(circumference)));

  Position prev_p(-1, -1);
  for (float degree = 0.0f; degree < 360.0f; degree += degree_step) {
    Position p = ellipse.get_positionAt(degree);
    if (isValidPosition(p) == false) {
      continue;
    }

    if (p.x != prev_p.x && p.y != prev_p.y) {
      positions.push_back(p);
      prev_p = p;
    }
  }

  PositionF center = ellipse.get_center();

  int circumference_length = int(positions.size());
  int aligned_count = 0;

  unsigned char* buffer = image_->buffer();

  for (auto p : positions) {
    int offset = p.y * width_ + p.x;

    int p00 = int(buffer[offset]);
    int p01 = int(buffer[offset + 1]);
    int p10 = int(buffer[offset + width_]);
    int p11 = int(buffer[offset + width_ + 1]);

    float gx = (p01 - p00 + p11 - p10) / 2.0f;
    float gy = (p10 - p00 + p11 - p01) / 2.0f;

    PositionF point_vector(p.x - center.x, p.y - center.y);
    float level_line_angle = atan2(point_vector.y, point_vector.x);
    float ellipse_angle = ellipse.angle();

    float new_aspect_x =
        cos(level_line_angle - ellipse_angle) / ellipse.major_length();
    float new_aspect_y =
        sin(level_line_angle - ellipse_angle) / ellipse.minor_length();

    level_line_angle = atan2(new_aspect_y, new_aspect_x) + ellipse_angle;

    float tangent1 = atan2(gy, gx);
    float tangent2 = atan2(-gy, -gx);

    float angle_diff = std::min(abs(tangent1 - level_line_angle),
                                abs(tangent2 - level_line_angle));

    if (angle_diff <= M_PI / 8.0f) {
      aligned_count++;
    }
  }

  float nfa = getCircleNFA(circumference_length, aligned_count);

  if (nfa <= 1.0f) {
    return true;
  } else {
    return false;
  }
}

float EDCircle::getCircleNFA(int circumference_length, int aligned_count) {
  double N = pow(sqrt(width_ * height_), 5.0);

  double factorial = 0.0f;
  for (auto i = aligned_count; i <= circumference_length; ++i) {
    int diff1 = circumference_length - i;
    int diff2 = circumference_length - diff1;
    int bigger_diff = std::max(diff1, diff2);
    int smaller_diff = std::min(diff1, diff2);

    double _f = 0.0f;

    for (auto n = bigger_diff + 1; n <= circumference_length; ++n) {
      _f += log(n);
    }
    for (auto n = 2; n <= smaller_diff; ++n) {
      _f -= log(n);
    }

    _f += log(pow(precision_, double(i)) *
              pow(1.0f - precision_, double(circumference_length - i)));

    _f = exp(_f);
    factorial += _f;
  }

  return N * factorial;
}

void EDCircle::ExtractArcs() {
  minimum_line_length_ = int(
      round(-4.0f * log(sqrt(float(width_) * float(height_))) / log(0.125f)));

  arcs_.clear();

  std::vector<Line> lines;

  for (const auto& edge : edge_segments_) {
    std::vector<Line> lines = ExtractLineSegments(edge);
    std::vector<std::vector<Line>> arc_candidates = ExtractArcCandidates(lines);

    for (const auto& candidate : arc_candidates) {
      Circle circle = Circle::FitFromEdgeSegment(candidate);

      if (circle.fitting_error() <= 1.5f) {
        Arc arc(candidate);
        arcs_.push_back(arc);
        continue;
      }

      auto search_begin = candidate.begin();
      auto search_end = candidate.begin();
      search_end += 3;

      bool is_found = false;

      while (search_begin != candidate.end()) {
        std::vector<Line> chunk_of_candidate;
        chunk_of_candidate.insert(chunk_of_candidate.end(), search_begin,
                                  search_end);

        Circle circle = Circle::FitFromEdgeSegment(chunk_of_candidate);

        if (circle.fitting_error() > 1.5f && chunk_of_candidate.size() >= 3) {
          if (is_found == true) {
            search_end--;

            std::vector<Line> new_arc_lines;
            new_arc_lines.insert(new_arc_lines.end(), search_begin, search_end);

            Arc arc(new_arc_lines);
            arcs_.push_back(arc);

            search_begin = search_end;
            is_found = false;
            continue;
          } else {
            search_begin++;

            if (std::distance(search_begin, candidate.end()) < 3) {
              break;
            } else {
              is_found = false;
              search_end = search_begin;
              search_end += 3;
              continue;
            }
          }
        }

        if (search_end == candidate.end()) {
          if (circle.fitting_error() <= 1.5f) {
            std::vector<Line> new_arc_lines;
            new_arc_lines.insert(new_arc_lines.end(), search_begin, search_end);

            Arc arc(new_arc_lines);
            arcs_.push_back(arc);
          }

          break;
        }
        is_found = true;
        search_end++;
      }
    }
  }
}
