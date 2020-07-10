#include <memory>

#include <opencv2/core.hpp>


class Image {
 public:
  Image(std::size_t width, std::size_t height, const unsigned char* buffer);
  static Image FromMat(cv::Mat& image);

 private:
  std::size_t width_;
  std::size_t height_;
  std::vector<unsigned char> buffer_;
};