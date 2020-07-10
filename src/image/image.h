#include <memory>
#include <opencv2/core.hpp>

template <typename T>
class Image {
 public:
  Image(std::size_t width, std::size_t height, const T* buffer) {
    width_ = width;
    height_ = height;
    buffer_.insert(buffer_.end(), buffer, buffer + (width_ * height_));
  }
  Image() = delete;

 private:
  std::size_t width_;
  std::size_t height_;
  std::vector<T> buffer_;
};
