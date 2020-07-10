#include <opencv2/highgui.hpp>
#include "image.h"


int main (int argc, char *argv[]) {
    cv::Mat input_image = cv::imread("Lenna.png");

    Image image = Image::FromMat(input_image);

    return 0;
}