#ifndef EDPF_H_
#define EDPF_H_

#include "edge_drawing.h"
#include "image/gray_image.h"

class EDPF : public EdgeDrawing {
 public:
  EDPF();

 public:
  void DetectEdge(GrayImage& image);

 public:
  static float GradientThreshold() { return (sqrt(8 * 8 + 8 * 8)); }

 protected:
  void SortAnchors();
  void PrepareNFA();
  void ValidateSegments();
  bool IsValidSegment(EdgeSegment& segment);
  float get_NFA(float magnitude, int segment_length);

 protected:
  std::vector<std::pair<float, float>> magnitude_histogram_;
  int N_p = 0;
};

#endif
