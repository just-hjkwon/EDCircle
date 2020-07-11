#ifndef EDLINE_H_
#define EDLINE_H_

#include "EDPF.h"
#include "image/gray_image.h"

class LineSegment {
 public:
  EdgeSegment edges;
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
  LineSegment FitLine(const EdgeSegment& segment);
  void ExtractLine();
  std::vector<LineSegment> ExtractLineSegments(const EdgeSegment segment);

  float ComputeFitError(const LineSegment& line, const EdgeSegment& segment);
  float ComputeFitError(const LineSegment& line, const Position& pos);

  bool isValidLineSegment(LineSegment line);

 protected:
  std::vector<LineSegment> line_segments_;

  float getSegmentNFA(int segment_length, int aligned_count);

 protected:
  int minimum_line_length_ = 0.0f;
  float aligned_degree_treshold_ = 0.0f;
  float precision_ = 0.0f;
};

#endif
