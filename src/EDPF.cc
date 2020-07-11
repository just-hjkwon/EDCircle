#include "EDPF.h"

#include <algorithm>

EDPF::EDPF() : EdgeDrawing(EDPF::GradientThreshold(), 0.0f, 1) {}

void EDPF::DetectEdge(GrayImage &image) {
  width_ = image.width();
  height_ = image.height();

  PrepareEdgeMap(image);
  ExtractAnchor();
  SortAnchors();
  ConnectingAnchors();
  PrepareNFA();
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

  for (auto y = 0; y < height_; ++y) {
    auto magnitude_ptr = magnitude_->buffer() + (width_ * y);

    for (auto x = 0; x < width_; ++x) {
      bool found = false;

      for (auto &bin : magnitude_histogram_) {
        if (bin.first == magnitude_ptr[x]) {
          bin.second++;
          found = true;
          break;
        }
      }

      if (found == false) {
        magnitude_histogram_.push_back(
            std::pair<float, int>(magnitude_ptr[x], 1));
      }
    }
  }

  std::sort(magnitude_histogram_.begin(), magnitude_histogram_.end(),
            [](const std::pair<float, int> &a, const std::pair<float, int> &b) {
              return a.first < b.first;
            });

  magnitude_histogram_.shrink_to_fit();
}
