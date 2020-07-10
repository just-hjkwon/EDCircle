#ifndef IMAGE__FILTERS_H_
#define IMAGE__FILTERS_H_

#include "../types.h"
#include "float_image.h"

class Filter : public FloatImage {
 public:
  Filter(std::size_t filter_size, const float* buffer)
      : FloatImage(filter_size, filter_size, buffer),
        center_(int(roundf(float(filter_size) / 2.0f)),
                int(roundf(float(filter_size) / 2.0f))){};

  template <typename T>
  float DoFilter(const T* image_ptr, Position position, std::size_t image_width,
                 std::size_t image_height);

  Position center() { return center_; };

 protected:
  Position center_;
};

class FilterFactory {
 public:
  static Filter CreateGaussianFilter(std::size_t filter_size, float sigma);
  static Filter PrewittXFilter();
  static Filter PrewittYFilter();
};

template <typename T>
inline float Filter::DoFilter(const T* image_ptr, Position position,
                              std::size_t image_width,
                              std::size_t image_height) {
  float value = 0.0f;

  int filter_y_offset = center_.y - height_;

  for (auto y = 0; y < height_; ++y, ++filter_y_offset) {
    int y_offset = std::min(std::max(0, position.y + filter_y_offset),
                            int(image_height) - 1);

    int filter_x_offset = center_.x - width_;

    for (auto x = 0; x < width_; ++x, ++filter_x_offset) {
      int x_offset = std::min(std::max(0, position.x + filter_x_offset),
                              int(image_width) - 1);
      float image_value = float(image_ptr[(y_offset * image_width) + x_offset]);
      float filter_value = buffer_[y * width_ + x];

      value += image_value * filter_value;
    }
  }

  return value;
}

#endif
