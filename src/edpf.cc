#include "edpf.h"

#include <algorithm>
#include <chrono>
#include <iostream>

EDPF::EDPF() : EdgeDrawing(EDPF::GradientThreshold(), 0.0f, 1) {}

void EDPF::DetectEdge(GrayImage &image) {
  width_ = image.width();
  height_ = image.height();

  std::chrono::system_clock::time_point start;
  std::chrono::duration<double> sec;

  start = std::chrono::system_clock::now();
  PrepareEdgeMap(image);
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDPF::PrepareEdgeMap Elapsed time: " << sec.count() * 1000.0f
            << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  ExtractAnchor();
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDPF::ExtractAnchor Elapsed time: " << sec.count() * 1000.0f
            << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  SortAnchors();
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDPF::SortAnchors Elapsed time: " << sec.count() * 1000.0f
            << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  ConnectingAnchors();
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDPF::ConnectingAnchors Elapsed time: " << sec.count() * 1000.0f
            << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  PrepareNFA();
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDPF::PrepareNFA Elapsed time: " << sec.count() * 1000.0f
            << " ms" << std::endl;

  start = std::chrono::system_clock::now();
  ValidateSegments();
  sec = std::chrono::system_clock::now() - start;
  std::cout << "EDPF::ValidateSegments Elapsed time: " << sec.count() * 1000.0f
            << " ms" << std::endl;
}

void EDPF::SortAnchors() {
  std::sort(anchors_.begin(), anchors_.end(),
            [=](const Edgel &a, const Edgel &b) -> bool {
              return a.magnitude > b.magnitude;
            });
}

void EDPF::PrepareNFA() {
  magnitude_cumulative_distribution_.clear();
  magnitude_cumulative_distribution_.reserve(width_ * height_);
  std::chrono::system_clock::time_point start;
  std::chrono::duration<double> sec;

  start = std::chrono::system_clock::now();

  std::vector<float> magnitudes;
  magnitudes.insert(magnitudes.end(), magnitude_->buffer(),
                    magnitude_->buffer() + width_ * height_);

  std::sort(magnitudes.begin(), magnitudes.end());

  float last_value = 0.0f;
  int current_index = -1;
  int zero_count = 0;
  for (auto m : magnitudes) {
    if (m == 0.0f) {
      zero_count++;
      continue;
    }

    if (m != last_value) {
      magnitude_cumulative_distribution_.push_back(std::make_pair(m, 1.0f));
      current_index++;
      last_value = m;
    } else {
      magnitude_cumulative_distribution_[current_index].second += 1.0f;
    }
  }
  int count = magnitudes.size() - zero_count;

  std::sort(magnitude_cumulative_distribution_.begin(), magnitude_cumulative_distribution_.end(),
            [](const std::pair<float, float> &a,
               const std::pair<float, float> &b) { return a.first > b.first; });

  if (magnitude_cumulative_distribution_.size() > 0) {
    magnitude_cumulative_distribution_[0].second /= float(count);
    magnitude_cumulative_distribution_table_.insert(
        std::make_pair(magnitude_cumulative_distribution_[0].first,
                       magnitude_cumulative_distribution_[0].second));
    for (int i = 1; i < magnitude_cumulative_distribution_.size(); ++i) {
      magnitude_cumulative_distribution_[i].second =
          magnitude_cumulative_distribution_[i - 1].second +
          (magnitude_cumulative_distribution_[i].second / float(count));

      magnitude_cumulative_distribution_table_.insert(
          std::make_pair(magnitude_cumulative_distribution_[i].first,
                         magnitude_cumulative_distribution_[i].second));
    }

    magnitude_cumulative_distribution_.shrink_to_fit();
  }

  N_p = 0;
  for (const auto &segment : edge_segments_) {
    N_p += (segment.size() * (segment.size() - 1));
  }
  N_p /= 2;
}

void EDPF::ValidateSegments() {
  std::list<EdgeSegment> valid_edge_segments;

  for (auto &segment : edge_segments_) {
    int segment_length = segment.size();

    if (segment_length <= 1) {
      continue;
    }

    if (segment_length <= 2) {
      auto min_it = std::min_element(segment.begin(), segment.end(),
                                     [](const Edgel &a, const Edgel &b) {
                                       return a.magnitude < b.magnitude;
                                     });

      if (IsValidSegment(min_it->magnitude, segment_length) == true) {
        EdgeSegment valid_edge_segment;
        valid_edge_segment.splice(valid_edge_segment.end(), segment,
                                  segment.begin(), segment.end());

        valid_edge_segments.push_back(valid_edge_segment);
        continue;
      }
    }

    std::list<std::pair<EdgeSegment::iterator, EdgeSegment::iterator>>
        valid_candidates;
    valid_candidates.push_back(std::make_pair(segment.begin(), segment.end()));

    while (valid_candidates.size() > 0) {
      auto sub_segment_begin_to_end = valid_candidates.front();
      valid_candidates.pop_front();

      int sub_segment_length = std::distance(sub_segment_begin_to_end.first,
                                             sub_segment_begin_to_end.second);

      if (sub_segment_length <= 1) {
        continue;
      }

      auto min_position = std::min_element(sub_segment_begin_to_end.first,
                                           sub_segment_begin_to_end.second,
                                           [](const auto &a, const auto &b) {
                                             return a.magnitude < b.magnitude;
                                           });

      if (IsValidSegment(min_position->magnitude, sub_segment_length) == true) {
        EdgeSegment valid_edge_segment;
        valid_edge_segment.splice(valid_edge_segment.end(), segment,
                                  sub_segment_begin_to_end.first,
                                  sub_segment_begin_to_end.second);
        valid_edge_segments.push_back(valid_edge_segment);
        continue;
      }

      int left_distance =
          std::distance(sub_segment_begin_to_end.first, min_position);
      int right_distance = sub_segment_length - left_distance - 1;

      if (left_distance > 1) {
        valid_candidates.push_back(
            std::make_pair(sub_segment_begin_to_end.first, min_position));
      }

      if (right_distance > 1) {
        min_position++;
        valid_candidates.push_back(
            std::make_pair(min_position, sub_segment_begin_to_end.second));
      }
    }
  }

  edge_segments_ = valid_edge_segments;
}

bool EDPF::IsValidSegment(EdgeSegment &segment) {
  auto min_it = std::min_element(
      segment.begin(), segment.end(),
      [](const auto &a, const auto &b) { return a.magnitude < b.magnitude; });

  float NPA = get_NFA(min_it->magnitude, int(segment.size()));
  if (NPA < 1.0) {
    return true;
  } else {
    return false;
  }
}

bool EDPF::IsValidSegment(float min_value, int segment_size) {
  float NPA = get_NFA(min_value, segment_size);
  if (NPA < 1.0) {
    return true;
  } else {
    return false;
  }
}

float EDPF::get_NFA(float magnitude, int segment_length) {
  float H = magnitude_cumulative_distribution_table_[magnitude];

  float NFA = float(N_p) * exp(float(segment_length) * log(H));
  return NFA;
}
