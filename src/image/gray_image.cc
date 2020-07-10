#include "gray_image.h"


void GrayImage::ApplyFilter(Filter &filter) {
  auto filter_center = filter.center();
  auto filter_width = filter.width();
  auto filter_height = filter.height();
  int filter_y_start = filter_center.y - filter_height;
  int filter_x_start = filter_center.x - filter_width;

  std::vector<unsigned char> filtered_buffer;
  filtered_buffer.resize(buffer_.size());
  
  auto source_ptr = buffer_.data();
  auto filtered_ptr = filtered_buffer.data();

  for (auto y = 0; y < height_; ++y) {
    auto filtered_y_ptr = filtered_ptr + (y * width_);

    for (auto x = 0; x < width_; ++x) {
      float value =
          filter.DoFilter(source_ptr, Position(x, y), width_, height_);
      filtered_y_ptr[x] = unsigned char(std::min(255, std::max(0, int(value))));
      //std::vector<unsigned char> flattened_patch;
      //flattened_patch.reserve(filter_width * filter_height);

      //for (auto fy = filter_y_start, i = 0; i < filter_height; ++fy, ++i) {
      //  auto source_y = (source_y_ptr + (fy * int(width_)));
      //  flattened_patch.insert(flattened_patch.end(), source_y,
      //                         source_y + filter_width);
      //}

      //float filtered_value = filter.DoFilter(flattened_patch);
      //filtered_y_ptr[x] =
      //    unsigned char(std::min(255, std::max(0, int(filtered_value))));
    }
  }

  buffer_ = filtered_buffer;
}

cv::Mat GrayImage::toMat() {
  return cv::Mat(cv::Size(width_, height_), CV_8UC1, buffer_.data()).clone();
}


GrayImage GrayImage::FromMat(cv::Mat &cv_image) {
  if (cv_image.type() != CV_8UC1) {
    throw std::invalid_argument("Input image should have the type of CV_8UC1.");
  }

  std::size_t width = cv_image.cols;
  std::size_t height = cv_image.rows;
  unsigned char *buffer = cv_image.data;

  return GrayImage(width, height, buffer);
}
