#include "ed_circle.h"

#include "circle.h"

EDCircle::EDCircle() { circle_fitting_error_threshold_ = 1.5f; }

void EDCircle::DetectCircle(GrayImage& image) {
  DetectEdge(image);

  std::vector<Circle> circles;

  cv::Mat c = cv::Mat::zeros(height_, width_, CV_8UC1);
  for (auto edge_segment : edge_segments_) {
    if (isClosedEdgeSegment(edge_segment) == true) {
      Circle circle = Circle::FitFromEdgeSegment(edge_segment);

      if (circle.fitting_error() < circle_fitting_error_threshold_) {
        circles.push_back(circle);
      }
    }
  }
}

bool EDCircle::isClosedEdgeSegment(const EdgeSegment& edge_segment) {
  auto first_edge = edge_segment.front();
  auto last_edge = edge_segment.back();

  int x_diff = abs(first_edge.first.x - last_edge.first.x);
  int y_diff = abs(first_edge.first.y - last_edge.first.y);

  if (x_diff <= 1 && y_diff <= 1) {
    return true;
  } else {
    return false;
  }
}
