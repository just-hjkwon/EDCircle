#ifndef IMAGE__FILTER_H_
#define IMAGE__FILTER_H_

#include "image.h"

class Filter {
 public:
  static void Gaussian(GrayImage &image, GrayImage &filtered_image,
                             std::size_t size, float sigma);
  static void Sobel(GrayImage &image, IntImage &gx, IntImage &gy,
                    FloatImage &magnitude);
  static void Prewitt(GrayImage &image, IntImage &gx, IntImage &gy,
                    FloatImage &magnitude);

 protected:
  static std::vector<float> CreateGaussianFilterBuffer(std::size_t size, float sigma);

};
#endif