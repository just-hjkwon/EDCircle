#include "gray_image.h"

void GrayImage::ApplyFilter(Filter &filter) {
  GrayImage filtered_image = MakeGrayFilteredImage(filter);
  buffer_ = filtered_image.buffer_;
}

GrayImage GrayImage::MakeGrayFilteredImage(Filter &filter) {
  auto filter_center = filter.center();
  auto filter_width = filter.width();
  auto filter_height = filter.height();
  int filter_y_start = filter_center.y - filter_height;
  int filter_x_start = filter_center.x - filter_width;

  std::vector<unsigned char> filtered_buffer;
  filtered_buffer.resize(buffer_.size());

  auto source_ptr = buffer_.data();
  auto filtered_ptr = filtered_buffer.data();
  auto filter_ptr = filter.buffer();

#if 0
  for (auto y = 0; y < height_; ++y) {
    auto filtered_y_ptr = filtered_ptr + (y * width_);

    for (auto x = 0; x < width_; ++x) {
      float value =
          filter.DoFilter(source_ptr, Position(x, y), width_, height_);
      filtered_y_ptr[x] = unsigned char(std::min(255, std::max(0, int(value))));
    }
  }
#else
  auto y_end = height_ - filter_height;
  auto x_end = width_ - filter_width;

  auto image_ptr = buffer_;
  filtered_ptr += (width_ * filter_center.y + filter_center.x);

  for (auto y = 0; y <= y_end; ++y) {
    for (auto x = 0; x <= x_end; ++x) {
      float value = 0.0f;

      for (auto fy = 0; fy < filter_height; ++fy) {
        std::size_t image_offset = width_ * (y + fy) + x;
        std::size_t filter_offset = filter_width * fy;
        for (auto fx = 0; fx < filter_width; ++fx) {
          value +=
              image_ptr[image_offset + fx] * filter_ptr[filter_offset + fx];
        }
      }

      filtered_ptr[width_ * y + x] =
          unsigned char(std::min(std::max(int(value), 0), 255));
    }
  }

  auto margin_y = height_ - (filter_height - filter_center.y - 1);
  auto margin_x = width_ - (filter_width - filter_center.x - 1);

  filtered_ptr = filtered_buffer.data();

  for (auto y = 0; y < height_; ++y) {
    for (auto x = 0; x < width_; ++x) {
      if (filter_center.y <= y && y < margin_y && filter_center.x <= x &&
          x < margin_x) {
        continue;
      }

      float value = 0.0f;
      for (int fy = -filter_center.y, fyy = 0;
           fy < -filter_center.y + filter_height; ++fy, ++fyy) {
        for (int fx = -filter_center.x, fxx = 0;
             fx < -filter_center.x + filter_width; ++fx, ++fxx) {
          int image_x = std::min(std::max(x + fx, 0), int(width_ - 1));
          int image_y = std::min(std::max(y + fy, 0), int(height_ - 1));

          value += image_ptr[(width_ * (image_y)) + image_x] *
                   filter_ptr[(filter_width * fyy) + fxx];
        }
      }

      filtered_ptr[width_ * y + x] =
          unsigned char(std::min(std::max(int(value), 0), 255));
    }
  }
#endif

  return GrayImage(width_, height_, filtered_buffer.data());
}

FloatImage GrayImage::MakeFloatFilteredImage(Filter &filter) {
  auto filter_center = filter.center();
  auto filter_width = filter.width();
  auto filter_height = filter.height();
  int filter_y_start = filter_center.y - filter_height;
  int filter_x_start = filter_center.x - filter_width;

  std::vector<float> filtered_buffer;
  filtered_buffer.resize(buffer_.size());

  auto source_ptr = buffer_.data();
  auto filtered_ptr = filtered_buffer.data();

  for (auto y = 0; y < height_; ++y) {
    auto filtered_y_ptr = filtered_ptr + (y * width_);

    for (auto x = 0; x < width_; ++x) {
      filtered_y_ptr[x] =
          filter.DoFilter(source_ptr, Position(x, y), width_, height_);
    }
  }

  return FloatImage(width_, height_, filtered_buffer.data());
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
