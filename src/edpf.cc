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
            [=](const Position &a, const Position &b) -> bool {
              float magnitude_a = magnitudeAt(a);
              float magnitude_b = magnitudeAt(b);

              return magnitude_a > magnitude_b;
            });
}

void EDPF::PrepareNFA() {
  magnitude_histogram_.clear();
  magnitude_histogram_.reserve(width_ * height_);
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
      magnitude_histogram_.push_back(std::make_pair(m, 1.0f));
      current_index++;
      last_value = m;
    } else {
      magnitude_histogram_[current_index].second += 1.0f;
    }
  }
  int count = magnitudes.size() - zero_count;
  for (auto &bin : magnitude_histogram_) {
    bin.second /= float(count);
  }

  std::sort(magnitude_histogram_.begin(), magnitude_histogram_.end(),
            [](const std::pair<float, float> &a,
               const std::pair<float, float> &b) { return a.first > b.first; });

  magnitude_histogram_.shrink_to_fit();

  N_p = 0;
  for (const auto &segment : edge_segments_) {
    N_p += (segment.size() * (segment.size() - 1) / 2);
  }
}

void EDPF::ValidateSegments() {
  std::list<EdgeSegment> valid_edge_semgments;

  for (auto it = edge_segments_.begin(); it != edge_segments_.end();) {
    if (it->size() <= 1) {
      it = edge_segments_.erase(it);
    } else {
      it++;
    }
  }

  auto segment_it = edge_segments_.begin();
    for (auto &segment : edge_segments_) {

    std::list<EdgeSegment> validation_list;
    validation_list.push_back(segment);
    segment_it = edge_segments_.begin();

    while (validation_list.size() > 0) {
      EdgeSegment _segment = validation_list.front();
      validation_list.pop_front();

      if (_segment.size() <= 1) {
        continue;
      }

      auto min_it = std::min_element(
          _segment.begin(), _segment.end(),
          [](const auto &a, const auto &b) { return a.second < b.second; });

      if (IsValidSegment(min_it->second, int(_segment.size())) == true) {
        valid_edge_semgments.push_back(_segment);
        continue;
      }

      int left_distance = std::distance(_segment.begin(), min_it);
      int right_distance = _segment.size() - left_distance - 1;

      if (left_distance > 1) {
        EdgeSegment new_segment;
        new_segment.splice(new_segment.end(), _segment, _segment.begin(),
                           min_it);
        validation_list.push_back(new_segment);
      }

      if (right_distance > 1) {
        EdgeSegment new_segment;
        min_it++;
        new_segment.splice(new_segment.end(), _segment, min_it, _segment.end());
        validation_list.push_back(new_segment);
      }
    }
  }

  edge_segments_ = valid_edge_semgments;
}

bool EDPF::IsValidSegment(EdgeSegment &segment) {
  auto min_it = std::min_element(
      segment.begin(), segment.end(),
      [](const auto &a, const auto &b) { return a.second < b.second; });

  float NPA = get_NFA(float(min_it->second), int(segment.size()));
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
  float H = 0.0f;

  for (auto bin : magnitude_histogram_) {
    if (bin.first < magnitude) {
      break;
    }
    H += bin.second;
  }

  float NPA = float(N_p) * pow(H, float(segment_length));
  return NPA;
}
