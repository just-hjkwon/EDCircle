#include "gray_image.h"


GrayImage GrayImage::FromMat(cv::Mat &cv_image) {
  if (cv_image.type() != CV_8UC1) {
    throw std::invalid_argument("Input image should have the type of CV_8UC1.");
  }

  std::size_t width = cv_image.cols;
  std::size_t height = cv_image.rows;
  unsigned char *buffer = cv_image.data;

  return GrayImage(width, height, buffer);
}
