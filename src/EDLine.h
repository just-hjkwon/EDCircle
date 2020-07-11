#ifndef EDLINE_H_
#define EDLINE_H_

#include "EDPF.h"
#include "image/gray_image.h"

class Line {
 public:
  float parameters[2] = {0.0f, 0.0f};
  float fitting_error = 0.0f;
  bool is_parameter_of_x = true;
};

class EDLine : public EDPF {
 public:
  EDLine();

 public:
  void DetectLine(GrayImage& image);

 protected:
  Line FitLine(const EdgeSegment& segment);
  void ExtractLine();
  std::vector<Line> ExtractLineSegments(const EdgeSegment segment);

  float ComputeFitError(const Line& line, const EdgeSegment& segment);
  float ComputeFitError(const Line& line, const Position& pos);

 protected:
  int minimum_line_length_ = 0.0f;
};

#endif
