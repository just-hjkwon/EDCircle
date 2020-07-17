#ifndef IMAGE__GRAY_IMAGE_H_
#define IMAGE__GRAY_IMAGE_H_

#include <opencv2/core.hpp>

#include "image.h"


class GrayImage : public Image<unsigned char> {
 public:
  GrayImage(std::size_t width, std::size_t height)
      : Image<unsigned char>(width, height){};
  GrayImage(std::size_t width, std::size_t height, const unsigned char* buffer)
      : Image<unsigned char>(width, height, buffer){};

  cv::Mat toMat();

  static GrayImage FromMat(cv::Mat &cv_image);
};

#endif
