#ifndef EDGE_DRAWING_H_
#define EDGE_DRAWING_H_

#include <memory>

#include "image/float_image.h"
#include "image/gray_image.h"

class EdgeDrawing {
 public:
  EdgeDrawing(GrayImage& image, float magnitude_threshold);

 public:
  enum class EdgeDirection : unsigned char {
    VerticalEdge = 0,
    HorizontalEdge = 1
  };

 private:
  void PrepareEdgeMap(GrayImage& image, float magnitude_threshold);

 private:
  std::shared_ptr<FloatImage> x_gradient_;
  std::shared_ptr<FloatImage> y_gradient_;
  std::shared_ptr<FloatImage> magnitude_;
  std::shared_ptr<Image<unsigned char>> direction_map_;
};

#endif
