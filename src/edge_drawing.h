#ifndef EDGE_DRAWING_H_
#define EDGE_DRAWING_H_

#include <memory>

#include "image/image.h"
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
  void set_verbose(bool verbose);
  void DetectEdge(GrayImage& image);
  std::list<EdgeSegment> edge_segments();

 protected:
  void PrepareEdgeMap(GrayImage& image);
  void ExtractAnchor();
  void ConnectAnchor();
  Position FindNextConnectingPosition(Position current,
                                      ConnectingAim direction);

  float magnitudeAt(Position position);
  EdgeDirection directionAt(Position position);
  void set_direction(Position position, EdgeDirection direction);

  void set_edge(Position position, bool value);
  bool is_edge(Position position);

  std::size_t get_offset(Position position);
  bool isValidPosition(Position position);

 protected:
  std::size_t width_;
  std::size_t height_;

  float magnitude_threshold_;
  float anchor_threshold_;
  int anchor_extraction_interval_;

 protected:
  IntImage gx_;
  IntImage gy_;
  FloatImage magnitude_;
  Image<unsigned char> direction_map_;

  std::list<Edgel> anchors_;
  Image<unsigned char> edge_map_;
  std::list<EdgeSegment> edge_segments_;

  bool verbose_ = false;
};

#endif
