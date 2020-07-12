#ifndef PRIMITIVES__ARC_H_
#define PRIMITIVES__ARC_H_

#include <vector>

#include "../types.h"
#include "circle.h"
#include "line.h"

class Arc {
 public:
  Arc(const std::vector<Line> &lines);
  Arc(const std::vector<Line> &lines, const Circle &fitted_circle);

 protected:
  Circle fitted_circle_;
  std::vector<Line> lines_;
  float length_;
};

#endif