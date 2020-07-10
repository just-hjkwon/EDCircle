#ifndef TYPES_H_
#define TYPES_H_

template <typename T>
class Position_ {
 public:
  T x;
  T y;
};

class PositionF : public Position_<float> {};
class Position : public Position_<int> {};

#endif
