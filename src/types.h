#ifndef TYPES_H_
#define TYPES_H_

#include <list>

template <typename T>
class Position_ {
 public:
  Position_(T _x, T _y) {
    x = _x;
    y = _y;
  }

 public:
  float DistanceWith(const Position_<T>& other) {
    return sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
  }

 public:
  T x;
  T y;
};

class PositionF : public Position_<float> {
 public:
  PositionF(float x, float y) : Position_<float>(x, y){};
};

class Position : public Position_<int> {
 public:
  Position(int x, int y) : Position_<int>(x, y){};
};

#endif
