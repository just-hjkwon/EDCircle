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

  protected:
  std::vector<std::pair<float, int>> magnitude_histogram_;

};

#endif