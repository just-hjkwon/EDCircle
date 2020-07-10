#include <opencv2/highgui.hpp>

#include "image/gray_image.h"
#include "image/filters.h"


int main (int argc, char *argv[]) {
    cv::Mat input_image = cv::imread("Lenna.png", cv::IMREAD_GRAYSCALE);

    GrayImage image = GrayImage::FromMat(input_image);

    Filter gaussian_filter = FilterFactory::CreateGaussianFilter(5, 1.0);

    image.ApplyFilter(gaussian_filter);

    cv::Mat filtered = image.toMat();

    cv::imshow("image", input_image);
    cv::imshow("filtered", filtered);

    cv::waitKey(0);

    return 0;
}