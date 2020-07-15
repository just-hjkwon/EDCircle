#include <chrono>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ed_circle.h"
#include "ed_line.h"
#include "edge_drawing.h"
#include "edpf.h"
#include "image/filters.h"
#include "image/gray_image.h"

int main(int argc, char *argv[]) {
  cv::Mat input_image = cv::imread("download.jpg", cv::IMREAD_GRAYSCALE);
  GrayImage image = GrayImage::FromMat(input_image);

  std::chrono::system_clock::time_point start =
      std::chrono::system_clock::now();

  Filter gaussian_filter = FilterFactory::CreateGaussianFilter(5, 1.0);
  image.ApplyFilter(gaussian_filter);
  std::chrono::duration<double> sec = std::chrono::system_clock::now() - start;
  std::cout << "Elapsed time: " << sec.count() << std::endl;
  EDCircle ed_circle;
  ed_circle.DetectCircle(image);



  cv::Mat i;
  cv::cvtColor(input_image, i, cv::COLOR_GRAY2BGR);

  for (auto circle : ed_circle.circles_) {
    circle.Draw(i, cv::Scalar(0, 0, 255));
  }

  for (auto e : ed_circle.ellipses_) {
    e.Draw(i, cv::Scalar(0, 255, 255));
  }

  cv::imshow("i", i);
  cv::waitKey(0);

  return 0;
}