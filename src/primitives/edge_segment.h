#ifndef PRIMITIVES__EDGE_SEGMENT_H_
#define PRIMITIVES__EDGE_SEGMENT_H_

#include <list>
#include <opencv2/core.hpp>

#include "../types.h"

class EdgeSegment : public std::list<Edgel> {
 public:
  bool isClosed() const;
  void Draw(cv::Mat &image, cv::Scalar color);

};

#endif