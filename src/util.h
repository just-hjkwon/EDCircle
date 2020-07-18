#ifndef UTIL_H_
#define UTIL_H_

#include <chrono>
#include <opencv2/core.hpp>

#include "image/image.h"

#define STOPWATCHSTART(x)   \
  if (x) {                  \
    Util::StopwatchStart(); \
  }
#define STOPWATCHSTOP(x, prefix)        \
  if (x) {                              \
    Util::StopwatchStopAndPrint(prefix); \
  }

class Util {
 public:
  static cv::Mat toMat(GrayImage &image);
  static GrayImage FromMat(cv::Mat &cv_image);

  static void StopwatchStart();
  static void StopwatchStopAndPrint(std::string prefix);

 protected:
  static std::chrono::system_clock::time_point start_time_;
};

#endif