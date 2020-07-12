#ifndef ED_CIRCLE_H_
#define ED_CIRCLE_H_

#include "ed_line.h"
#include "image/gray_image.h"
#include "primitives/arc.h"
#include "primitives/circle.h"
#include "primitives/ellipse.h"

class EDCircle : public EDLine {
 public:
  EDCircle();

 public:
  void DetectCircle(GrayImage& image);

 protected:
  bool isClosedEdgeSegment(const EdgeSegment& edge_segment);
  void DetectCircleAndEllipseFromClosedEdgeSegment();
  void ExtractArcs();
  std::vector<std::vector<Line>> ExtractArcCandidates(
      const std::vector<Line>& line_segments);

 protected:
  std::vector<Circle> circles_;
  std::vector<Ellipse> ellipses_;
  std::vector<Arc> arcs_;

  float circle_fitting_error_threshold_;
  float ellipse_fitting_error_threshold_;
  float arc_line_angle_thresholds_[2];
};

#endif
