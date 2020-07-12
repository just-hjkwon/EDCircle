#ifndef ED_LINE_H_
#define ED_LINE_H_

#include "edpf.h"
#include "image/gray_image.h"
#include "primitives/line.h"

//class LineSegment {
// public:
//  EdgeSegment edges;
//
//  float parameters_[2] = {0.0f, 0.0f};
//  float fitting_error_ = 0.0f;
//  bool is_parameter_of_x = true;
//
// public:
//  // Position begin();
//  // Position end();
//};

class EDLine : public EDPF {
 public:
  EDLine();

 public:
  void DetectLine(GrayImage& image);

 protected:
  void ExtractLine();
  std::vector<Line> ExtractLineSegments(const EdgeSegment &segment);

  bool isValidLineSegment(const Line &line);

 public:
  std::vector<Line> lines_;

  float getSegmentNFA(int segment_length, int aligned_count);

 protected:
  int minimum_line_length_ = 0.0f;
  float aligned_degree_treshold_ = 0.0f;
  float precision_ = 0.0f;
};

#endif
