#ifndef ED_CIRCLE_H_
#define ED_CIRCLE_H_

#include "ed_line.h"
#include "image/image.h"
#include "primitives/arc.h"
#include "primitives/circle.h"
#include "primitives/ellipse.h"

class EDCircle : public EDLine {
 public:
  EDCircle();

 public:
  void DetectCircle(GrayImage& image);

  std::list<Circle> circles();
  std::list<Ellipse> ellipses();
  std::list<Arc> arcs();
  std::list<Arc> extended_arcs();

 protected:
  void DetectCircleAndEllipseFromClosedEdgeSegment();
  void ExtractArcs();
  std::vector<std::vector<Line>> ExtractArcCandidates(
      const std::vector<Line>& line_segments);
  void ExtendArcsAndDetectCircle();
  void ExtendArcsAndDetectEllipse();
  void ValidateCircleAndEllipse(GrayImage& image);
  bool isValidCircle(const Circle& circle, GrayImage& image);
  bool isValidEllipse(const Ellipse& ellipse, GrayImage& image);

  float getCircleNFA(int circumference_length, int aligned_count);

 protected:
  std::list<EdgeSegment> not_closed_edge_segmnets_;
  std::list<Circle> circles_;
  std::list<Ellipse> ellipses_;
  std::list<Arc> arcs_;
  std::list<Arc> extended_arcs_;

  float circle_fitting_error_threshold_;
  float ellipse_fitting_error_threshold_;
  float arc_line_angle_thresholds_[2];
};

#endif
