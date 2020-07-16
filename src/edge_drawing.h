#ifndef EDGE_DRAWING_H_
#define EDGE_DRAWING_H_

#include <memory>

#include "image/float_image.h"
#include "image/gray_image.h"
#include "primitives/edge_segment.h"

enum class EdgeDirection : unsigned char {
  VerticalEdge = 0,
  HorizontalEdge = 1
};

enum class ConnectingAim : unsigned char {
  Left = 0,
  Up = 1,
  Right = 2,
  Down = 3
};

class EdgeDrawing {
 public:
  EdgeDrawing(float magnitude_threshold, float anchor_threshold,
              int anchor_extraction_interval);

 public:
  void DetectEdge(GrayImage& image);

 protected:
  void PrepareEdgeMap(GrayImage& image);
  void ExtractAnchor();
  void ConnectingAnchors();
  Position FindNextConnectingPosition(Position current,
                                      ConnectingAim direction);

  float magnitudeAt(Position pos);
  EdgeDirection directionAt(Position pos);
  void set_direction(Position pos, EdgeDirection direction);

  void set_edge(Position pos, bool value);
  bool is_edge(Position pos);

  std::size_t get_offset(Position pos);
  bool isValidPosition(Position pos);

 protected:
  std::size_t width_;
  std::size_t height_;

  float magnitude_threshold_;
  float anchor_threshold_;
  int anchor_extraction_interval_;

 protected:
  std::shared_ptr<FloatImage> x_gradient_;
  std::shared_ptr<FloatImage> y_gradient_;
  std::shared_ptr<FloatImage> magnitude_;
  std::shared_ptr<Image<unsigned char>> direction_map_;

  std::vector<Position> anchors_;
  std::shared_ptr<Image<unsigned char>> edge_map_;

  std::list<EdgeSegment> edge_segments_;
};

#endif
