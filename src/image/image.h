#ifndef IMAGE__IMAGE_H_
#define IMAGE__IMAGE_H_

#include <memory>
#include <opencv2/core.hpp>

template <typename T>
class Image {
 public:
  Image(std::size_t width, std::size_t height)
      : Image(width, height, nullptr){};
  Image(std::size_t width, std::size_t height, const T* buffer) {
    width_ = width;
    height_ = height;

    if (buffer == nullptr) {
      buffer_.resize(width * height);
    } else {
      buffer_.insert(buffer_.end(), buffer, buffer + (width_ * height_));
    }
  };
  Image() = delete;

 public:
  void Reset(std::size_t width, std::size_t height) {
    if (width_ * height_ != width * height) {
      buffer_.resize(width * height);
    }

    std::fill(buffer_.begin(), buffer_.end(), 0);

    width_ = width;
    height_ = height;
  };

  std::size_t width() { return width_; };
  std::size_t height() { return height_; };
  T* buffer() { return buffer_.data(); };

 protected:
  std::size_t width_;
  std::size_t height_;
  std::vector<T> buffer_;
};

using IntImage = Image<int>;
using FloatImage = Image<float>;

#endif
