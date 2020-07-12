#ifndef PRIMITIVES__ARC_H_
#define PRIMITIVES__ARC_H_

#include <vector>

#include "../types.h"
#include "line.h"

class Arc {
 public:
  Arc(std::vector<Line> lines);

 protected:
  std::vector<Line> lines_;
};

#endif