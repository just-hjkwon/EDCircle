#include <opencv2/highgui.hpp>

#include "image/gray_image.h"


int main (int argc, char *argv[]) {
    cv::Mat input_image = cv::imread("Lenna.png", cv::IMREAD_GRAYSCALE);

    GrayImage image = GrayImage::FromMat(input_image);
    

    return 0;
}