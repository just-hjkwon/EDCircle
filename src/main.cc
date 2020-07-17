#include <chrono>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ed_circle.h"
#include "ed_line.h"
#include "edge_drawing.h"
#include "edpf.h"
#include "image/filter.h"
#include "image/image.h"
#include "primitives/circle.h"
#include "util.h"

struct Config {
 public:
  std::string filename;
  bool video_mode;
  bool display_info;
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
  if (argc < 3) {
    Config config{"", false, true};
    return config;
  }

  bool video_mode = false;
  std::string filename;
  bool error = false;
  bool display_info = false;

  for (int i = 1; i < argc; i++) {
    if (std::string("-v").compare(argv[i]) == 0) {
      video_mode = true;
      if (i + 1 < argc) {
        filename = argv[i + 1];
        i++;
      }
    } else if (std::string("-i").compare(argv[i]) == 0) {
      video_mode = false;
      if (i + 1 < argc) {
        filename = argv[i + 1];
        i++;
      }
    } else if (std::string("-d").compare(argv[i]) == 0) {
      display_info = true;
    } else {
      error = true;
    }
  }

  if (filename.compare("") == 0) {
    error = true;
  }

  if (error == true) {
    return Config{"", false, false, true};
  } else {
    return Config{filename, video_mode, display_info, false};
  }
}

void DetectCircle(cv::Mat &cv_image) {
  cv::Mat cv_gray_image;
  if (cv_image.type() == CV_8UC3) {
    cv::cvtColor(cv_image, cv_gray_image, cv::COLOR_BGR2GRAY);
  } else {
    cv_gray_image = cv_image;
  }

  GrayImage image = Util::FromMat(cv_gray_image);
  GrayImage gaussian_filtered(image.width(), image.height());
  Filter::Gaussian(image, gaussian_filtered, 5, 1.0);

  EDCircle ed_circle;
  ed_circle.DetectCircle(gaussian_filtered);
}
