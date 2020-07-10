#include "image.h"

Image::Image(std::size_t width, std::size_t height, const unsigned char* buffer) {
  width_ = width;
  height_ = height;
  buffer_.insert(buffer_.end(), buffer, buffer + (width_ * height_));
}

Image Image::FromMat(cv::Mat& image) {
  std::size_t width = image.cols;
  std::size_t height = image.rows;

  const unsigned char *image_buffer = image.data;

  return Image(width, height, image_buffer);
}
