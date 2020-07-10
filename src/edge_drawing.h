#ifndef EDGE_DRAWING_H_
#define EDGE_DRAWING_H_

#include <memory>

#include "image/float_image.h"
#include "image/gray_image.h"

class EdgeDrawing {
 public:
  EdgeDrawing(GrayImage& image, float magnitude_threshold,
              float anchor_threshold, int anchor_extraction_interval);

 public:
  enum class EdgeDirection : unsigned char {
    VerticalEdge = 0,
    HorizontalEdge = 1
  };

 private:
  void PrepareEdgeMap(GrayImage& image);
  void ExtractAnchor();
  void ConnectingAnchors();
  void DoSmartRouteToLeft(Position start_position);
  void DoSmartRouteToRight(Position start_position);
  void DoSmartRouteToUp(Position start_position);
  void DoSmartRouteToDown(Position start_position);

 private:
  std::size_t width_;
  std::size_t height_;

  float magnitude_threshold_;
  float anchor_threshold_;
  int anchor_extraction_interval_;

 private:
  std::shared_ptr<FloatImage> x_gradient_;
  std::shared_ptr<FloatImage> y_gradient_;
  std::shared_ptr<FloatImage> magnitude_;
  std::shared_ptr<Image<unsigned char>> direction_map_;

  std::vector<Position> anchors_;
  std::shared_ptr<Image<unsigned char>> edge_map_;
};

#endif
