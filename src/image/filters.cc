#include "filters.h"

#include <iostream>

Filter FilterFactory::PrewittXFilter() {
  std::vector<float> buffer = {-1.0f, 0.0f,  1.0f, -1.0f, 0.0f,
                               1.0f,  -1.0f, 0.0f, 1.0f};

  const std::size_t filter_size = 3;
  return Filter(filter_size, buffer.data());
}

Filter FilterFactory::PrewittYFilter() {
  std::vector<float> buffer = {-1.0f, -1.0f, -1.0f, 0.0f, 0.0f,
                               0.0f,  1.0f,  1.0f,  1.0f};

  const std::size_t filter_size = 3;
  return Filter(filter_size, buffer.data());
}
