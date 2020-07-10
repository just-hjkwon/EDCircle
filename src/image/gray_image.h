#include <opencv2/core.hpp>

#include "image.h"


class GrayImage : public Image<unsigned char> {
 public:
  GrayImage(std::size_t width, std::size_t height, const unsigned char* buffer)
      : Image<unsigned char>(width, height, buffer){};

  static GrayImage FromMat(cv::Mat &cv_image);
};
