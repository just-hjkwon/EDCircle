#include "ed_circle.h"

#include "primitives/circle.h"
#include "primitives/ellipse.h"

EDCircle::EDCircle() {
  circle_fitting_error_threshold_ = 1.5f;
  ellipse_fitting_error_threshold_ = 1.5f;
}

void EDCircle::DetectCircle(GrayImage& image) {
  DetectEdge(image);

  std::vector<Circle> circles;
  std::vector<Ellipse> ellipses;

  for (auto edge_segment : edge_segments_) {
    if (isClosedEdgeSegment(edge_segment) == true) {
      Circle circle = Circle::FitFromEdgeSegment(edge_segment);

      if (circle.fitting_error() < circle_fitting_error_threshold_) {
        circles.push_back(circle);
        continue;
      }

      Ellipse ellipse = Ellipse::FitFromEdgeSegment(edge_segment);

      if (ellipse.fitting_error() < ellipse_fitting_error_threshold_) {
        ellipses.push_back(ellipse);
        continue;
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
