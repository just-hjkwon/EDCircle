#include "edge_segment.h"

bool EdgeSegment::isClosed() const {
  auto first_edge = front();
  auto last_edge = back();

  int x_diff = abs(first_edge.position.x - last_edge.position.x);
  int y_diff = abs(first_edge.position.y - last_edge.position.y);

  if (x_diff <= 1 && y_diff <= 1) {
    return true;
  } else {
    return false;
  }
}

void EdgeSegment::Draw(cv::Mat& image, cv::Scalar color) {
  int type = image.type();

  if (type == CV_8UC1) {
    for (auto it = begin(); it != end(); it++) {
      image.at<unsigned char>(it->position.y, it->position.x) = color[0];
    }

  } else if (type == CV_8UC3) {
    for (auto it = begin(); it != end(); it++) {
      image.at<cv::Vec3b>(it->position.y, it->position.x) = cv::Vec3b(color[0], color[1], color[2]);
    }
  }
}
