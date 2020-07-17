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
  bool verbose;
  bool error;
};

void print_help();
void print_invalid_input_file(std::string filename);
Config parse_args(int argc, char *argv[]);
void DetectCircle(cv::Mat &cv_image, bool verbose);

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

    std::cout << "Press 'q' to exit." << std::endl;

    while (true) {
      cv::Mat frame;
      video.read(frame);
      if (frame.empty() == true) {
        break;
      }

      DetectCircle(frame, config.verbose);

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

    DetectCircle(image, config.verbose);

    cv::waitKey(0);
  }
}

void print_help() {
  std::cout << "Usage: EDCircle [-m|-i] [video filename|image filename]"
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
  bool verbose = false;

  for (int i = 1; i < argc; i++) {
    if (std::string("-m").compare(argv[i]) == 0) {
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
    } else if (std::string("-v").compare(argv[i]) == 0) {
      verbose = true;
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
    return Config{filename, video_mode, verbose, false};
  }
}

void DetectCircle(cv::Mat &cv_image, bool verbose) {
  cv::Mat cv_gray_image;
  if (cv_image.type() == CV_8UC3) {
    cv::cvtColor(cv_image, cv_gray_image, cv::COLOR_BGR2GRAY);
  } else {
    cv_gray_image = cv_image;
  }

  GrayImage image = Util::FromMat(cv_gray_image);
  GrayImage gaussian_filtered(image.width(), image.height());
  STOPWATCHSTART(verbose)
  Filter::Gaussian(image, gaussian_filtered, 5, 1.0);
  STOPWATCHSTOP(verbose, "Filter::Gaussian - ")

  EDCircle ed_circle;
  ed_circle.set_verbose(verbose);
  ed_circle.DetectCircle(gaussian_filtered);

  if (verbose == true) {
    cv::Mat edge_image = cv::Mat::zeros(cv_image.size(), CV_8UC1);
    cv::Mat lines_image = cv_image.clone();
    cv::Mat arcs_image = cv_image.clone();
    cv::Mat extended_arcs_image = cv_image.clone();

    auto edge_segments = ed_circle.edge_segments();
    auto lines = ed_circle.lines();
    auto arcs = ed_circle.arcs();
    auto extended_arcs = ed_circle.arcs();

    for (auto edge_segment : edge_segments) {
      edge_segment.Draw(edge_image, cv::Scalar(255, 255, 255));
    }

    for (auto line : lines) {
      line.Draw(lines_image, cv::Scalar(255, 255, 0));
    }

    for (auto arc : arcs) {
      arc.Draw(arcs_image, cv::Scalar(255, 255, 0));
    }

    for (auto extended_arc : extended_arcs) {
      extended_arc.Draw(extended_arcs_image, cv::Scalar(255, 255, 0));
    }

    cv::imshow("Edge", edge_image);
    cv::imshow("Lines", lines_image);
    cv::imshow("Arcs", arcs_image);
    cv::imshow("Extended arcs", extended_arcs_image);
  }

  cv::Mat circle_and_ellipse_image = cv_image.clone();

  auto circles = ed_circle.circles();
  auto ellipses = ed_circle.ellipses();

  for (auto circle : circles) {
    circle.Draw(circle_and_ellipse_image, cv::Scalar(255, 255, 0));
  }
  for (auto ellipse : ellipses) {
    ellipse.Draw(circle_and_ellipse_image, cv::Scalar(0, 255, 255));
  }

  cv::imshow("Circles and Ellipse", circle_and_ellipse_image);
}
