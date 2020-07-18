#ifndef ED_LINE_H_
#define ED_LINE_H_

#include "edpf.h"
#include "image/image.h"
#include "primitives/line.h"

class EDLine : public EDPF {
 public:
  EDLine();

 public:
  void DetectLine(GrayImage &image);

  std::list<Line> lines();

 protected:
  void ExtractLine();
  std::vector<Line> ExtractLinesFromEdgeSegment(const EdgeSegment &segment);

  bool IsValidLine(const Line &line);
  float getLineNFA(int segment_length, int aligned_count);

 protected:
  std::list<Line> lines_;

 protected:
  int minimum_line_length_ = 0.0f;
  float aligned_degree_treshold_ = 0.0f;
  float precision_ = 0.0f;
};

#endif
