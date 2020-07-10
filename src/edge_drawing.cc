#include "edge_drawing.h"

#include "image/filters.h"


EdgeDrawing::EdgeDrawing(GrayImage& image, float magnitude_threshold) {
  PrepareEdgeMap(image, magnitude_threshold);
}

void EdgeDrawing::PrepareEdgeMap(GrayImage& image, float magnitude_threshold) {
  Filter prewitt_x = FilterFactory::PrewittXFilter();
  Filter prewitt_y = FilterFactory::PrewittYFilter();

  FloatImage x_gradient = image.MakeFloatFilteredImage(prewitt_x);
  FloatImage y_gradient = image.MakeFloatFilteredImage(prewitt_y);

  std::size_t width = image.width();
  std::size_t height = image.height();

  x_gradient_ =
      std::make_shared<FloatImage>(width, height, x_gradient.buffer());
  y_gradient_ =
      std::make_shared<FloatImage>(width, height, y_gradient.buffer());

  std::vector<float> magnitude(width * height);
  std::vector<unsigned char> direction(width * height);

  for (auto y = 0; y < height; ++y) {
    auto x_ptr = x_gradient_->buffer() + (width * y);
    auto y_ptr = x_gradient_->buffer() + (width * y);

    auto magnitude_ptr = magnitude.data() + (width * y);
    auto direction_ptr = direction.data() + (width * y);
     
    for (auto x = 0; x < width; ++x) {
      float magnitude = sqrt((x_ptr[x] * x_ptr[x]) + (y_ptr[x] * y_ptr[x]));
      if (magnitude >= magnitude_threshold) {
        magnitude_ptr[x] = magnitude;
      } else {
        magnitude_ptr[x] = 0.0f;
      }

      if (abs(x_ptr[x]) >= abs(y_ptr[x])) {
        direction_ptr[x] = (unsigned char)EdgeDirection::VerticalEdge;
      } else {
        direction_ptr[x] = (unsigned char)EdgeDirection::HorizontalEdge;
      }
    }
  }

  magnitude_ = std::make_shared<FloatImage>(width, height, magnitude.data());
  direction_map_ =
      std::make_shared<Image<unsigned char>>(width, height, direction.data());
}
