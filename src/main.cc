#include "main.h"

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
#include "primitives/circle.h"

struct Config {
 public:
  std::string filename;
  bool video_mode;
  bool error;
};

void print_help();
void print_invalid_input_file(std::string filename);
Config parse_args(int argc, char *argv[]);
void DetectCircle(cv::Mat &cv_image);

int main(int argc, char *argv[]) {
  Config config = parse_args(argc, argv);
  if (config.error == true) {
    print_help();
    return -1;
  }

  if (config.video_mode == true) {
    cv::VideoCapture video;
    bool is_opened = video.open(config.filename);

    if (is_opened == false) {
      print_invalid_input_file(config.filename);
      return -1;
    }

    while (true) {
      cv::Mat frame;
      video.read(frame);
      if (frame.empty() == true) {
        break;
      }

      DetectCircle(frame);

      cv::imshow("EVideo", frame);
      char pressed_key = cv::waitKey(1);
      if (pressed_key == 'q') {
        break;
      }
    }
  } else {
    cv::Mat image = cv::imread(config.filename);
    if (image.empty() == true) {
      print_invalid_input_file(config.filename);
      return -1;
    }

    DetectCircle(image);

    cv::imshow("Image", image);
    cv::waitKey(0);
  }
}

void print_help() {
  std::cout << "Usage: EDCircle [-v|-i] [video filename|image filename]"
            << std::endl;
}

void print_invalid_input_file(std::string filename) {
  std::cout << "Invalid input file: " << filename << std::endl;
}

Config parse_args(int argc, char *argv[]) {
  if (argc != 3) {
    return Config{"", false, true};
  }

  bool video_mode = false;
  if (std::string("-v").compare(argv[1]) == 0) {
    video_mode = true;
  } else if (std::string("-i").compare(argv[1]) == 0) {
    video_mode = false;
  } else {
    return Config{"", false, true};
  }

  std::string filename(argv[2]);

  return Config{filename, video_mode, false};
}

void DetectCircle(cv::Mat &cv_image) {
  cv::Mat cv_gray_image;
  if (cv_image.type() == CV_8UC3) {
    cv::cvtColor(cv_image, cv_gray_image, cv::COLOR_BGR2GRAY);
  } else {
    cv_gray_image = cv_image;
  }

  GrayImage image = GrayImage::FromMat(cv_gray_image);

  std::chrono::system_clock::time_point start;
  std::chrono::duration<double> sec;

  start = std::chrono::system_clock::now();
  Filter gaussian_filter = FilterFactory::CreateGaussianFilter(5, 1.0);
  image.ApplyFilter(gaussian_filter);
  sec = std::chrono::system_clock::now() - start;
  std::cout << "Gaussian Elapsed time: " << sec.count() * 1000.0f << " ms"
            << std::endl;

  start = std::chrono::system_clock::now();
  EDCircle ed_circle;
  ed_circle.DetectCircle(image);
  sec = std::chrono::system_clock::now() - start;
  std::cout << "DetectCircle Elapsed time: " << sec.count() * 1000.0f << " ms"
            << std::endl;
}
