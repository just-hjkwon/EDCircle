#include "util.h"

#include <iostream>

std::chrono::system_clock::time_point Util::start_time_;

cv::Mat Util::toMat(GrayImage &image) {
  return cv::Mat(cv::Size(image.width(), image.height()), CV_8UC1,
                 (void *)image.buffer())
      .clone();
}

GrayImage Util::FromMat(cv::Mat &cv_image) {
  if (cv_image.type() != CV_8UC1) {
    throw std::invalid_argument("Input image should have the type of CV_8UC1.");
  }

  std::size_t width = cv_image.cols;
  std::size_t height = cv_image.rows;
  unsigned char *buffer = cv_image.data;

  return GrayImage(width, height, buffer);
}

void Util::StopwatchStart() {
  Util::start_time_ = std::chrono::system_clock::now();
}

void Util::StopwatchStopAndPrint(std::string prefix) {
  std::chrono::duration<double> elapsed_time =
      std::chrono::system_clock::now() - start_time_;
  std::cout << prefix << elapsed_time.count() * 1000.0f << " ms" << std::endl;
}
