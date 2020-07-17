#ifndef IMAGE__IMAGE_H_
#define IMAGE__IMAGE_H_

#include <memory>
#include <opencv2/core.hpp>

template <typename T>
class Image {
 public:
  Image(std::size_t width, std::size_t height);
  Image(std::size_t width, std::size_t height, const T* buffer);
  Image() = delete;

 public:
  void Reset(std::size_t width, std::size_t height);
 
  std::size_t width();
  std::size_t height();
  T* buffer();

 protected:
  std::size_t width_;
  std::size_t height_;
  std::vector<T> buffer_;
};

template <typename T>
Image<T>::Image(std::size_t width, std::size_t height)
    : Image(width, height, nullptr) {}

template <typename T>
Image<T>::Image(std::size_t width, std::size_t height, const T* buffer) {
  width_ = width;
  height_ = height;

  if (buffer == nullptr) {
    buffer_.resize(width * height);
  } else {
    buffer_.insert(buffer_.end(), buffer, buffer + (width_ * height_));
  }
}

template <typename T>
void Image<T>::Reset(std::size_t width, std::size_t height) {
  if (width_ * height_ != width * height) {
    buffer_.resize(width * height);
  }

  std::fill(buffer_.begin(), buffer_.end(), 0);

  width_ = width;
  height_ = height;
}

template <typename T>
std::size_t Image<T>::width() {
  return width_;
}

template <typename T>
std::size_t Image<T>::height() {
  return height_;
}

template <typename T>
T* Image<T>::buffer() {
  return buffer_.data();
}

using GrayImage = Image<unsigned char>;
using IntImage = Image<int>;
using FloatImage = Image<float>;

#endif