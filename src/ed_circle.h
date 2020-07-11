#ifndef ED_CIRCLE_H_
#define ED_CIRCLE_H_

#include "ed_line.h"
#include "image/gray_image.h"

class EDCircle : public EDLine {
 public:
  EDCircle();

 public:
  void DetectCircle(GrayImage& image);

 protected:
  bool isClosedEdgeSegment(const EdgeSegment& edge_segment);

 protected:
  float circle_fitting_error_threshold_;
};

#endif
