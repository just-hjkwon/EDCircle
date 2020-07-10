#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "edge_drawing.h"
#include "image/filters.h"
#include "image/gray_image.h"


int main(int argc, char *argv[]) {
  cv::Mat input_image = cv::imread("Lenna.png", cv::IMREAD_GRAYSCALE);

  GrayImage image = GrayImage::FromMat(input_image);

  Filter gaussian_filter = FilterFactory::CreateGaussianFilter(5, 1.0);

  image.ApplyFilter(gaussian_filter);

  cv::Mat filtered = image.toMat();

  cv::imshow("image", input_image);
  cv::imshow("filtered", filtered);

  EdgeDrawing edge_drawing(image, 36.0f, 8.0f, 1);

  cv::waitKey(0);

  return 0;
}