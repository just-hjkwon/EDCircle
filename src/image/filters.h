#ifndef IMAGE__FILTERS_H_
#define IMAGE__FILTERS_H_

#include "float_image.h"
#include "../types.h"


class Filter : FloatImage {
 public:
  Filter(std::size_t filter_size, const float *buffer)
      : FloatImage(filter_size, filter_size, buffer) {
    center_.x = int(roundf(float(filter_size) / 2.0f));
    center_.y = int(roundf(float(filter_size) / 2.0f));
  };

 Position center() { return center_; };

 private:
  Position center_;
};

class FilterFactory {
 public:
  static Filter CreateGaussianFilter(std::size_t filter_size, float sigma);
};

#endif