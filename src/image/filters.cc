#include "filters.h"

#include <iostream>

Filter FilterFactory::CreateGaussianFilter(std::size_t filter_size,
                                           float sigma) {
  std::vector<float> buffer(filter_size * filter_size);

  int center = int(roundf(float(filter_size - 1) / 2.0f));

  auto buffer_ptr = buffer.data();

  float sigma_denominator = 2.0f * sigma * sigma;
  float filter_sum = 0.0f;

  for (auto y = 0; y < filter_size; ++y) {
    auto y_ptr = buffer_ptr + (filter_size * y);

    for (auto x = 0; x < filter_size; ++x) {
      float x_term = float((center - x) * (center - x));
      float y_term = float((center - y) * (center - y));

      y_ptr[x] = exp(-1.0f * float(x_term + y_term) / sigma_denominator);
      filter_sum += y_ptr[x];
    }
  }

  for (auto& f : buffer) {
    f /= filter_sum;
  }

  return Filter(filter_size, buffer.data());
}

Filter FilterFactory::PrewittXFilter() {
  std::vector<float> buffer = { -1.0f,  0.0f, 1.0f, 
                                -1.0f,  0.0f, 1.0f, 
                                -1.0f,  0.0f, 1.0f};

  const std::size_t filter_size = 3;
  return Filter(filter_size, buffer.data());
}

Filter FilterFactory::PrewittYFilter() {
  std::vector<float> buffer = { -1.0f,  -1.0f, -1.0f,
                                0.0f, 0.0f, 0.0f,
                               1.0f, 1.0f, 1.0f };

  const std::size_t filter_size = 3;
  return Filter(filter_size, buffer.data());
}

Filter FilterFactory::SobelXFilter() {
  std::vector<float> buffer = { -1.0f, 0.0f, 1.0f, 
                                -2.0f, 0.0f, 2.0f,
                                -1.0f, 0.0f, 1.0f};

  const std::size_t filter_size = 3;
  return Filter(filter_size, buffer.data());
}

Filter FilterFactory::SobelYFilter() {
  std::vector<float> buffer = { -1.0f, -2.0f, -1.0f,
                                0.0f, 0.0f, 0.0f,
                                1.0f,  2.0f,  1.0f};

  const std::size_t filter_size = 3;
  return Filter(filter_size, buffer.data());
}